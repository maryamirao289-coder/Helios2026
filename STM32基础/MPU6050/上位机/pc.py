import math
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class MPU6050App:
    def __init__(self, root):
        self.root = root

        self.root.title("MPU6050 Attitude Monitor")
        self.root.geometry("1100x720")
        self.root.minsize(900, 620)

        # -------------------------------------------------
        # Serial
        # -------------------------------------------------
        self.serial_port = None
        self.serial_thread = None
        self.serial_running = False

        self.rx_queue = queue.Queue()

        # -------------------------------------------------
        # Data
        # -------------------------------------------------
        self.start_time = time.monotonic()

        self.time_data = deque(maxlen=1000)
        self.roll_data = deque(maxlen=1000)
        self.pitch_data = deque(maxlen=1000)
        self.yaw_data = deque(maxlen=1000)

        self.rx_frame_count = 0
        self.rx_error_count = 0

        # -------------------------------------------------
        # Tk variables
        # -------------------------------------------------
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")

        self.roll_var = tk.StringVar(value="0.000 °")
        self.pitch_var = tk.StringVar(value="0.000 °")
        self.yaw_var = tk.StringVar(value="0.000 °")

        self.kp_var = tk.StringVar(value="2.00")
        self.ki_var = tk.StringVar(value="0.02")

        self.status_var = tk.StringVar(value="Disconnected")

        self.rx_count_var = tk.StringVar(value="0")
        self.rx_error_var = tk.StringVar(value="0")

        # -------------------------------------------------
        # Build GUI
        # -------------------------------------------------
        self.create_widgets()

        self.refresh_ports()

        # GUI update loop
        self.root.after(50, self.update_gui)

        # Window close callback
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def create_widgets(self):
        # =================================================
        # Top connection frame
        # =================================================
        connection_frame = ttk.LabelFrame(
            self.root,
            text="Serial Connection",
            padding=10
        )
        connection_frame.pack(
            fill="x",
            padx=10,
            pady=10
        )

        ttk.Label(
            connection_frame,
            text="Port:"
        ).grid(
            row=0,
            column=0,
            padx=5,
            pady=5
        )

        self.port_combo = ttk.Combobox(
            connection_frame,
            textvariable=self.port_var,
            width=15,
            state="readonly"
        )
        self.port_combo.grid(
            row=0,
            column=1,
            padx=5,
            pady=5
        )

        ttk.Button(
            connection_frame,
            text="Refresh",
            command=self.refresh_ports
        ).grid(
            row=0,
            column=2,
            padx=5,
            pady=5
        )

        ttk.Label(
            connection_frame,
            text="Baud:"
        ).grid(
            row=0,
            column=3,
            padx=5,
            pady=5
        )

        self.baud_combo = ttk.Combobox(
            connection_frame,
            textvariable=self.baud_var,
            width=12,
            state="readonly",
            values=[
                "9600",
                "115200",
                "230400",
                "460800",
                "921600"
            ]
        )
        self.baud_combo.grid(
            row=0,
            column=4,
            padx=5,
            pady=5
        )

        self.connect_button = ttk.Button(
            connection_frame,
            text="Connect",
            command=self.toggle_connection
        )
        self.connect_button.grid(
            row=0,
            column=5,
            padx=10,
            pady=5
        )

        ttk.Label(
            connection_frame,
            text="Status:"
        ).grid(
            row=0,
            column=6,
            padx=(20, 5),
            pady=5
        )

        ttk.Label(
            connection_frame,
            textvariable=self.status_var
        ).grid(
            row=0,
            column=7,
            padx=5,
            pady=5
        )

        # =================================================
        # Main area
        # =================================================
        main_frame = ttk.Frame(self.root)
        main_frame.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=(0, 10)
        )

        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=4)
        main_frame.rowconfigure(0, weight=1)

        # =================================================
        # Left panel
        # =================================================
        left_frame = ttk.Frame(main_frame)
        left_frame.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 10)
        )

        # -------------------------------------------------
        # Attitude display
        # -------------------------------------------------
        attitude_frame = ttk.LabelFrame(
            left_frame,
            text="Attitude",
            padding=15
        )
        attitude_frame.pack(
            fill="x",
            pady=(0, 10)
        )

        ttk.Label(
            attitude_frame,
            text="Roll"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=8
        )

        ttk.Label(
            attitude_frame,
            textvariable=self.roll_var,
            font=("Arial", 14)
        ).grid(
            row=0,
            column=1,
            sticky="e",
            padx=10
        )

        ttk.Label(
            attitude_frame,
            text="Pitch"
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=8
        )

        ttk.Label(
            attitude_frame,
            textvariable=self.pitch_var,
            font=("Arial", 14)
        ).grid(
            row=1,
            column=1,
            sticky="e",
            padx=10
        )

        ttk.Label(
            attitude_frame,
            text="Yaw"
        ).grid(
            row=2,
            column=0,
            sticky="w",
            pady=8
        )

        ttk.Label(
            attitude_frame,
            textvariable=self.yaw_var,
            font=("Arial", 14)
        ).grid(
            row=2,
            column=1,
            sticky="e",
            padx=10
        )

        attitude_frame.columnconfigure(1, weight=1)

        # -------------------------------------------------
        # Mahony gain settings
        # -------------------------------------------------
        gain_frame = ttk.LabelFrame(
            left_frame,
            text="Mahony Parameters",
            padding=15
        )
        gain_frame.pack(
            fill="x",
            pady=(0, 10)
        )

        ttk.Label(
            gain_frame,
            text="Kp:"
        ).grid(
            row=0,
            column=0,
            padx=5,
            pady=8
        )

        ttk.Entry(
            gain_frame,
            textvariable=self.kp_var,
            width=12
        ).grid(
            row=0,
            column=1,
            padx=5,
            pady=8
        )

        ttk.Label(
            gain_frame,
            text="Ki:"
        ).grid(
            row=1,
            column=0,
            padx=5,
            pady=8
        )

        ttk.Entry(
            gain_frame,
            textvariable=self.ki_var,
            width=12
        ).grid(
            row=1,
            column=1,
            padx=5,
            pady=8
        )

        ttk.Button(
            gain_frame,
            text="Apply Gains",
            command=self.send_gains
        ).grid(
            row=2,
            column=0,
            columnspan=2,
            sticky="ew",
            padx=5,
            pady=(10, 5)
        )

        # -------------------------------------------------
        # Statistics
        # -------------------------------------------------
        statistics_frame = ttk.LabelFrame(
            left_frame,
            text="Statistics",
            padding=15
        )
        statistics_frame.pack(
            fill="x"
        )

        ttk.Label(
            statistics_frame,
            text="Valid frames:"
        ).grid(
            row=0,
            column=0,
            sticky="w",
            pady=5
        )

        ttk.Label(
            statistics_frame,
            textvariable=self.rx_count_var
        ).grid(
            row=0,
            column=1,
            sticky="e",
            padx=10
        )

        ttk.Label(
            statistics_frame,
            text="Parse errors:"
        ).grid(
            row=1,
            column=0,
            sticky="w",
            pady=5
        )

        ttk.Label(
            statistics_frame,
            textvariable=self.rx_error_var
        ).grid(
            row=1,
            column=1,
            sticky="e",
            padx=10
        )

        statistics_frame.columnconfigure(1, weight=1)

        # =================================================
        # Plot
        # =================================================
        plot_frame = ttk.LabelFrame(
            main_frame,
            text="Real-Time Attitude",
            padding=5
        )
        plot_frame.grid(
            row=0,
            column=1,
            sticky="nsew"
        )

        self.figure = Figure(
            figsize=(8, 6),
            dpi=100
        )

        self.ax = self.figure.add_subplot(111)

        self.ax.set_title(
            "Roll / Pitch / Yaw"
        )

        self.ax.set_xlabel(
            "Time (s)"
        )

        self.ax.set_ylabel(
            "Angle (deg)"
        )

        self.ax.grid(True)

        self.roll_line, = self.ax.plot(
            [],
            [],
            label="Roll"
        )

        self.pitch_line, = self.ax.plot(
            [],
            [],
            label="Pitch"
        )

        self.yaw_line, = self.ax.plot(
            [],
            [],
            label="Yaw"
        )

        self.ax.legend(
            loc="upper right"
        )

        self.canvas = FigureCanvasTkAgg(
            self.figure,
            master=plot_frame
        )

        self.canvas.get_tk_widget().pack(
            fill="both",
            expand=True
        )

    def refresh_ports(self):
        ports = []

        for port in serial.tools.list_ports.comports():
            ports.append(port.device)

        self.port_combo["values"] = ports

        if ports:
            if self.port_var.get() not in ports:
                self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def toggle_connection(self):
        if self.serial_port is None:
            self.connect_serial()
        else:
            self.disconnect_serial()

    def connect_serial(self):
        port = self.port_var.get()

        if not port:
            messagebox.showwarning(
                "Serial",
                "No serial port selected."
            )
            return

        try:
            baud = int(
                self.baud_var.get()
            )
        except ValueError:
            messagebox.showerror(
                "Serial",
                "Invalid baud rate."
            )
            return

        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )

        except serial.SerialException as exc:
            self.serial_port = None

            messagebox.showerror(
                "Serial Error",
                str(exc)
            )

            return

        self.serial_running = True

        self.start_time = time.monotonic()

        self.time_data.clear()
        self.roll_data.clear()
        self.pitch_data.clear()
        self.yaw_data.clear()

        self.rx_frame_count = 0
        self.rx_error_count = 0

        self.serial_thread = threading.Thread(
            target=self.serial_reader,
            daemon=True
        )

        self.serial_thread.start()

        self.connect_button.config(
            text="Disconnect"
        )

        self.status_var.set(
            "Connected: {} @ {}".format(
                port,
                baud
            )
        )

    def disconnect_serial(self):
        self.serial_running = False

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass

        self.serial_port = None

        self.connect_button.config(
            text="Connect"
        )

        self.status_var.set(
            "Disconnected"
        )

    def serial_reader(self):
        while self.serial_running:
            if self.serial_port is None:
                break

            try:
                raw_line = self.serial_port.readline()

                if not raw_line:
                    continue

                line = raw_line.decode(
                    "ascii",
                    errors="ignore"
                ).strip()

                if not line:
                    continue

                if line.startswith("GAIN_OK,"):
                    ack_parts = line.split(",")

                    if len(ack_parts) == 3:
                        try:
                            ack_kp = float(
                                ack_parts[1]
                            )

                            ack_ki = float(
                                ack_parts[2]
                            )

                        except ValueError:
                            self.rx_queue.put(
                                ("error", None)
                            )
                            continue

                        self.rx_queue.put(
                            (
                                "gain_ack",
                                (
                                    ack_kp,
                                    ack_ki
                                )
                            )
                        )

                        continue
                parts = line.split(",")

                if len(parts) != 3:
                    self.rx_queue.put(
                        ("error", None)
                    )
                    continue

                try:
                    roll = float(parts[0])
                    pitch = float(parts[1])
                    yaw = float(parts[2])

                except ValueError:
                    self.rx_queue.put(
                        ("error", None)
                    )
                    continue

                if not (
                    math.isfinite(roll)
                    and math.isfinite(pitch)
                    and math.isfinite(yaw)
                ):
                    self.rx_queue.put(
                        ("error", None)
                    )
                    continue

                timestamp = (
                    time.monotonic()
                    - self.start_time
                )

                self.rx_queue.put(
                    (
                        "data",
                        (
                            timestamp,
                            roll,
                            pitch,
                            yaw
                        )
                    )
                )

            except (
                serial.SerialException,
                OSError
            ) as exc:
                self.rx_queue.put(
                    (
                        "serial_error",
                        str(exc)
                    )
                )
                break

    def update_gui(self):
        latest_sample = None
        plot_changed = False

        while True:
            try:
                message_type, payload = (
                    self.rx_queue.get_nowait()
                )

            except queue.Empty:
                break

            if message_type == "data":
                timestamp, roll, pitch, yaw = payload

                self.time_data.append(
                    timestamp
                )

                self.roll_data.append(
                    roll
                )

                self.pitch_data.append(
                    pitch
                )

                self.yaw_data.append(
                    yaw
                )

                self.rx_frame_count += 1

                latest_sample = (
                    roll,
                    pitch,
                    yaw
                )

                plot_changed = True

            elif message_type == "gain_ack":
                ack_kp, ack_ki = payload

                self.kp_var.set(
                    "{:.2f}".format(
                        ack_kp
                    )
                )

                self.ki_var.set(
                    "{:.4f}".format(
                        ack_ki
                    )
                )

                self.status_var.set(
                    "Confirmed: Kp={:.6f}, Ki={:.6f}".format(
                        ack_kp,
                        ack_ki
                    )
                )

            elif message_type == "error":
                self.rx_error_count += 1

            elif message_type == "serial_error":
                self.status_var.set(
                    "Serial error: {}".format(
                        payload
                    )
                )

                self.disconnect_serial()

        if latest_sample is not None:
            roll, pitch, yaw = latest_sample

            self.roll_var.set(
                "{:.3f} °".format(roll)
            )

            self.pitch_var.set(
                "{:.3f} °".format(pitch)
            )

            self.yaw_var.set(
                "{:.3f} °".format(yaw)
            )

        self.rx_count_var.set(
            str(self.rx_frame_count)
        )

        self.rx_error_var.set(
            str(self.rx_error_count)
        )

        if plot_changed:
            self.update_plot()

        self.root.after(
            50,
            self.update_gui
        )

    def update_plot(self):
        if not self.time_data:
            return

        x = list(self.time_data)

        roll = list(self.roll_data)
        pitch = list(self.pitch_data)
        yaw = list(self.yaw_data)

        self.roll_line.set_data(
            x,
            roll
        )

        self.pitch_line.set_data(
            x,
            pitch
        )

        self.yaw_line.set_data(
            x,
            yaw
        )

        current_time = x[-1]

        # Display approximately the latest 10 seconds
        if current_time > 10.0:
            self.ax.set_xlim(
                current_time - 10.0,
                current_time
            )
        else:
            self.ax.set_xlim(
                0.0,
                10.0
            )

        all_values = (
            roll
            + pitch
            + yaw
        )

        if all_values:
            minimum = min(all_values)
            maximum = max(all_values)

            margin = max(
                5.0,
                (maximum - minimum) * 0.1
            )

            if minimum == maximum:
                minimum -= 5.0
                maximum += 5.0

            self.ax.set_ylim(
                minimum - margin,
                maximum + margin
            )

        self.canvas.draw_idle()

    def send_gains(self):
        if (
            self.serial_port is None
            or not self.serial_port.is_open
        ):
            messagebox.showwarning(
                "Mahony Parameters",
                "Serial port is not connected."
            )
            return

        try:
            kp = float(
                self.kp_var.get()
            )

            ki = float(
                self.ki_var.get()
            )

        except ValueError:
            messagebox.showerror(
                "Mahony Parameters",
                "Kp and Ki must be valid numbers."
            )
            return

        if not (
            math.isfinite(kp)
            and math.isfinite(ki)
        ):
            messagebox.showerror(
                "Mahony Parameters",
                "Kp and Ki must be finite numbers."
            )
            return

        command = (
            "KP={:.6f},KI={:.6f}\n".format(
                kp,
                ki
            )
        )

        try:
            self.serial_port.write(
                command.encode("ascii")
            )

            self.status_var.set(
                "Sent: KP={:.6f}, KI={:.6f}".format(
                    kp,
                    ki
                )
            )

        except serial.SerialException as exc:
            messagebox.showerror(
                "Serial Error",
                str(exc)
            )

    def on_close(self):
        self.disconnect_serial()

        self.root.destroy()


def main():
    root = tk.Tk()

    MPU6050App(root)

    root.mainloop()


if __name__ == "__main__":
    main()