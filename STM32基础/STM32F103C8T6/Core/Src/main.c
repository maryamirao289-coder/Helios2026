/* USER CODE BEGIN Header */
/** 
  ****************************************************************************** 
  * @file           : main.c 
  * @brief          : Main program body 
  ****************************************************************************** 
  * @attention 
  * 
  * Copyright (c) 2026 STMicroelectronics. 
  * All rights reserved. 
  * 
  * This software is licensed under terms that can be found in the LICENSE file 
  * in the root directory of this software component. 
  * If no LICENSE file comes with this software, it is provided AS-IS. 
  * 
  ****************************************************************************** 
  */ 
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "protocol.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t uart_sync_byte = 0;

uint8_t uart_rx_frame[
    PROTOCOL_ATTITUDE_FRAME_LEN
];

volatile uint8_t uart_rx_state = 0;
volatile uint8_t uart_frame_pending = 0;

volatile HAL_StatusTypeDef uart_rx_start_status = HAL_ERROR;

Protocol_Attitude_t received_attitude;

uint16_t received_sequence = 0;

volatile Protocol_Status_t protocol_last_status = PROTOCOL_OK;

volatile uint32_t frame_ok_count = 0;
volatile uint32_t frame_crc_error_count = 0;
volatile uint32_t frame_other_error_count = 0;

volatile uint32_t frame_rate = 0;

uint32_t frame_rate_last_tick = 0;
uint32_t frame_rate_last_count = 0;

uint8_t retransmit_request_frame[
    PROTOCOL_RETRANSMIT_REQUEST_LEN
];

volatile uint8_t
    retransmit_request_pending = 0;

volatile uint16_t
    retransmit_request_sequence = 0;

volatile HAL_StatusTypeDef
    retransmit_request_tx_status = HAL_ERROR;

volatile uint32_t
    retransmit_request_tx_count = 0;

volatile uint32_t
    retransmit_request_tx_error_count = 0;


/* Wait for the requested frame to come back */
volatile uint8_t retransmit_waiting = 0;

volatile uint16_t
    retransmit_waiting_sequence = 0;

volatile uint32_t
    retransmit_success_count = 0;

char pc_attitude_buffer[64];

uint32_t pc_attitude_last_tick = 0;

volatile uint8_t pc_attitude_tx_busy = 0;

volatile HAL_StatusTypeDef
    pc_attitude_tx_status = HAL_ERROR;

volatile uint32_t
    pc_attitude_tx_count = 0;

volatile uint32_t
    pc_attitude_tx_busy_count = 0;

uint8_t pc_rx_byte = 0;

char pc_rx_line[64];

volatile uint8_t pc_rx_index = 0;
volatile uint8_t pc_command_pending = 0;

volatile HAL_StatusTypeDef
    pc_rx_start_status = HAL_ERROR;

volatile float pc_kp = 0.0f;
volatile float pc_ki = 0.0f;

volatile uint32_t
    pc_gain_cmd_count = 0;

volatile uint32_t
    pc_gain_cmd_error_count = 0;

uint8_t gains_tx_frame[
    PROTOCOL_SET_GAINS_FRAME_LEN
];

volatile uint16_t
    gains_tx_sequence = 0;

volatile HAL_StatusTypeDef
    gains_tx_status = HAL_ERROR;

volatile uint32_t
    gains_tx_count = 0;

volatile uint32_t
    gains_tx_error_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

uart_rx_state = 0;

uart_rx_start_status =
    HAL_UART_Receive_IT(
        &huart1,
        &uart_sync_byte,
        1
    );

frame_rate_last_tick =
    HAL_GetTick();

frame_rate_last_count =
    frame_ok_count;

pc_attitude_last_tick = HAL_GetTick();

pc_rx_start_status =
    HAL_UART_Receive_IT(
        &huart2,
        &pc_rx_byte,
        1
    );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) 
  { 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (uart_frame_pending)
		{
		    protocol_last_status =
		        Protocol_ParseAttitudeFrame(
		            uart_rx_frame,
		            PROTOCOL_ATTITUDE_FRAME_LEN,
		            &received_sequence,
		            &received_attitude
		        );
			if (protocol_last_status == PROTOCOL_OK)
{
    frame_ok_count++;


    /*
     * Was this the frame we requested again?
     */
    if ((retransmit_waiting != 0) &&
        (received_sequence ==
         retransmit_waiting_sequence))
    {
        retransmit_success_count++;

        retransmit_waiting = 0;
    }
}
else if (
    protocol_last_status ==
    PROTOCOL_ERROR_CRC
)
{
    frame_crc_error_count++;


    /*
     * Protocol_ParseAttitudeFrame()
     * already extracted SEQ before
     * checking CRC.
     */
    retransmit_request_sequence =
        received_sequence;

    retransmit_request_pending = 1;
}
else
{
    frame_other_error_count++;
}


		    uart_frame_pending = 0;

		    uart_rx_state = 0;


		    /* Wait for next frame header */
		    HAL_UART_Receive_IT(
		        &huart1,
		        &uart_sync_byte,
		        1
		    );
/*
 * Send retransmit request only after
 * RX has already been armed again.
 */
if (retransmit_request_pending)
{
    Protocol_BuildRetransmitRequest(
        retransmit_request_frame,
        retransmit_request_sequence
    );


    retransmit_request_tx_status =
        HAL_UART_Transmit(
            &huart1,
            retransmit_request_frame,
            PROTOCOL_RETRANSMIT_REQUEST_LEN,
            10
        );


    if (retransmit_request_tx_status ==
        HAL_OK)
    {
        retransmit_request_tx_count++;

        retransmit_waiting = 1;

        retransmit_waiting_sequence =
            retransmit_request_sequence;
    }
    else
    {
        retransmit_request_tx_error_count++;
    }


    retransmit_request_pending = 0;
}

		}


			/* Calculate actual frame rate */
		if ((HAL_GetTick() - frame_rate_last_tick) >= 1000)
		{
		    frame_rate =
		        frame_ok_count -
		        frame_rate_last_count;


		    frame_rate_last_count =
		        frame_ok_count;


		    frame_rate_last_tick += 1000;
		}


/*
 * Send Roll / Pitch / Yaw to PC at 100Hz
 */
if ((HAL_GetTick() - pc_attitude_last_tick) >= 10)
{
    int pc_attitude_length;


    pc_attitude_last_tick += 10;


    if (pc_attitude_tx_busy == 0)
    {
        pc_attitude_length =
            snprintf(
                pc_attitude_buffer,
                sizeof(pc_attitude_buffer),
                "%.3f,%.3f,%.3f\n",
                received_attitude.roll,
                received_attitude.pitch,
                received_attitude.yaw
            );


        if ((pc_attitude_length > 0) &&
            (pc_attitude_length <
             sizeof(pc_attitude_buffer)))
        {
            pc_attitude_tx_status =
                HAL_UART_Transmit_IT(
                    &huart2,
                    (uint8_t *)pc_attitude_buffer,
                    pc_attitude_length
                );


            if (pc_attitude_tx_status ==
                HAL_OK)
            {
                pc_attitude_tx_busy = 1;

                pc_attitude_tx_count++;
            }
        }
    }
    else
    {
        pc_attitude_tx_busy_count++;
    }
}


/*
 * Parse gain command from PC
 *
 * Example:
 * KP=2.50,KI=0.02
 */
if (pc_command_pending)
{
    float new_kp;
    float new_ki;


    pc_command_pending = 0;


    if (sscanf(
            pc_rx_line,
            "KP=%f,KI=%f",
            &new_kp,
            &new_ki
        ) == 2)
    {
        pc_kp = new_kp;
        pc_ki = new_ki;

        pc_gain_cmd_count++;


        /*
         * Build SET_GAINS protocol frame
         */
        Protocol_BuildSetGainsFrame(
            gains_tx_frame,
            gains_tx_sequence,
            pc_kp,
            pc_ki
        );


        /*
         * Send gain command to slave
         */
        gains_tx_status =
            HAL_UART_Transmit(
                &huart1,
                gains_tx_frame,
                PROTOCOL_SET_GAINS_FRAME_LEN,
                10
            );


        if (gains_tx_status == HAL_OK)
        {
            gains_tx_count++;

            gains_tx_sequence++;
        }
        else
        {
            gains_tx_error_count++;
        }
    }
    else
    {
        pc_gain_cmd_error_count++;
    }
}

  } 
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /*
         * State 0:
         * wait for 0xAA
         */
        if (uart_rx_state == 0)
        {
            if (uart_sync_byte == PROTOCOL_HEADER_1)
            {
                uart_rx_frame[0] =
                    uart_sync_byte;

                uart_rx_state = 1;
            }


            HAL_UART_Receive_IT(
                &huart1,
                &uart_sync_byte,
                1
            );
        }


        /*
         * State 1:
         * wait for 0x55
         */
        else if (uart_rx_state == 1)
        {
            if (uart_sync_byte == PROTOCOL_HEADER_2)
            {
                uart_rx_frame[1] =
                    uart_sync_byte;

                uart_rx_state = 2;


                /*
                 * Header is complete.
                 * Receive remaining 34 bytes.
                 */
                HAL_UART_Receive_IT(
                    &huart1,
                    &uart_rx_frame[2],
                    PROTOCOL_ATTITUDE_FRAME_LEN - 2
                );
            }
            else
            {
                /*
                 * Resynchronization
                 */
                if (uart_sync_byte == PROTOCOL_HEADER_1)
                {
                    uart_rx_frame[0] =
                        uart_sync_byte;

                    uart_rx_state = 1;
                }
                else
                {
                    uart_rx_state = 0;
                }


                HAL_UART_Receive_IT(
                    &huart1,
                    &uart_sync_byte,
                    1
                );
            }
        }


        /*
         * State 2:
         * complete frame received
         */
        else if (uart_rx_state == 2)
        {
            uart_frame_pending = 1;

            /*
             * Temporarily stop receiving.
             * main() parses the frame first.
             */
            uart_rx_state = 3;
        }
    }
		    else if (huart->Instance == USART2)
    {
        if (pc_rx_byte == '\n')
        {
            if (pc_rx_index > 0)
            {
                pc_rx_line[pc_rx_index] = '\0';

                pc_command_pending = 1;

                pc_rx_index = 0;
            }
        }
        else if (pc_rx_byte != '\r')
        {
            if (pc_rx_index <
                (sizeof(pc_rx_line) - 1))
            {
                pc_rx_line[pc_rx_index] =
                    (char)pc_rx_byte;

                pc_rx_index++;
            }
            else
            {
                /*
                 * Buffer overflow:
                 * discard current command
                 */
                pc_rx_index = 0;
            }
        }


        /*
         * Continue receiving next PC byte
         */
        HAL_UART_Receive_IT(
            &huart2,
            &pc_rx_byte,
            1
        );
    }
}


void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef *huart
)
{
    if (huart->Instance == USART2)
    {
        pc_attitude_tx_busy = 0;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */ 
  __disable_irq(); 
  while (1) 
  { 
  } 
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number, 
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */ 
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
