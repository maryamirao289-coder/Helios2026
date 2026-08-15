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
#include "mpu6050.h"
#include "mahony.h"
#include "protocol.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ATTITUDE_HISTORY_SIZE              16

#define PROTOCOL_TEST_CORRUPT_SEQUENCE     3000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
MPU6050_RawData_t imu_raw;
MPU6050_Data_t imu_data;
MPU6050_GyroBias_t gyro_bias;

Mahony_t mahony;

volatile HAL_StatusTypeDef imu_calib_status = HAL_ERROR;
volatile HAL_StatusTypeDef imu_init_status = HAL_ERROR;
volatile HAL_StatusTypeDef imu_read_status = HAL_ERROR;

volatile uint8_t imu_sample_flag = 0;

/* TIM2真正产生了多少次1ms事件 */
volatile uint32_t imu_timer_tick_count = 0;

/* MPU6050成功读取了多少次 */
volatile uint32_t imu_sample_count = 0;

/* main来不及处理时统计丢失的采样节拍 */
volatile uint32_t imu_overrun_count = 0;

volatile uint32_t imu_sample_rate = 0;

uint32_t rate_last_tick = 0;
uint32_t rate_last_sample_count = 0;


Protocol_Attitude_t attitude_tx_data;

uint8_t attitude_tx_frame[
    PROTOCOL_ATTITUDE_FRAME_LEN
];

volatile uint16_t attitude_tx_sequence = 0;

volatile uint32_t attitude_tx_count = 0;
volatile uint32_t attitude_tx_error_count = 0;

volatile HAL_StatusTypeDef
    attitude_tx_status = HAL_ERROR;

/* Attitude frame history for retransmission */
uint8_t attitude_history
    [ATTITUDE_HISTORY_SIZE]
    [PROTOCOL_ATTITUDE_FRAME_LEN];

uint16_t attitude_history_sequence[
    ATTITUDE_HISTORY_SIZE
];

uint8_t attitude_history_valid[
    ATTITUDE_HISTORY_SIZE
];

uint8_t attitude_history_write_index = 0;


/* Retransmit request RX */
uint8_t retransmit_sync_byte = 0;

uint8_t retransmit_rx_frame[
    PROTOCOL_SET_GAINS_FRAME_LEN
];

volatile uint8_t retransmit_rx_state = 0;
volatile uint8_t retransmit_request_pending = 0;

volatile uint8_t retransmit_rx_frame_length = 0;

volatile HAL_StatusTypeDef
    retransmit_rx_start_status = HAL_ERROR;

volatile Protocol_Status_t
    retransmit_request_status = PROTOCOL_OK;

volatile uint16_t
    retransmit_requested_sequence = 0;

volatile Protocol_Status_t
    set_gains_status = PROTOCOL_OK;

uint16_t set_gains_sequence = 0;

float set_gains_kp = 0.0f;
float set_gains_ki = 0.0f;

volatile uint32_t set_gains_count = 0;
volatile uint32_t set_gains_error_count = 0;


uint8_t gains_ack_frame[
    PROTOCOL_GAINS_ACK_FRAME_LEN
];

volatile uint8_t
    gains_ack_pending = 0;

volatile HAL_StatusTypeDef
    gains_ack_tx_status = HAL_ERROR;

volatile uint32_t
    gains_ack_tx_count = 0;

volatile uint32_t
    gains_ack_tx_error_count = 0;


/* Retransmit TX */
uint8_t retransmit_tx_frame[
    PROTOCOL_ATTITUDE_FRAME_LEN
];

volatile uint8_t retransmit_tx_pending = 0;
volatile uint8_t retransmit_tx_in_progress = 0;

volatile HAL_StatusTypeDef
    retransmit_tx_status = HAL_ERROR;

volatile uint32_t retransmit_request_count = 0;
volatile uint32_t retransmit_tx_count = 0;
volatile uint32_t retransmit_tx_error_count = 0;
volatile uint32_t retransmit_not_found_count = 0;


/*
 * Test switch:
 *
 * 1 = intentionally corrupt one frame
 *     to verify CRC retransmission
 *
 * After verification change this to 0.
 */
volatile uint8_t protocol_test_enable = 0;
volatile uint8_t protocol_test_done = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t Slave_FindHistoryFrame(
    uint16_t sequence,
    uint8_t *frame
)
{
    uint8_t i;


    for (i = 0;
         i < ATTITUDE_HISTORY_SIZE;
         i++)
    {
        if ((attitude_history_valid[i] != 0) &&
            (attitude_history_sequence[i] ==
             sequence))
        {
            memcpy(
                frame,
                attitude_history[i],
                PROTOCOL_ATTITUDE_FRAME_LEN
            );

            return 1;
        }
    }


    return 0;
}
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
imu_init_status = MPU6050_Init();

if (imu_init_status == HAL_OK)
{
    imu_calib_status =
        MPU6050_CalibrateGyro(
            &gyro_bias,
            1000
        );
}

if ((imu_init_status == HAL_OK) &&
    (imu_calib_status == HAL_OK))
{
	  Mahony_Init(
        &mahony,
        2.0f,
        0.02f
    );
	
	
    HAL_TIM_Base_Start_IT(&htim2);
	
	  /* 记录采样率统计的起始时间 */
    rate_last_tick = HAL_GetTick();

    /* 记录当前采样总数 */
    rate_last_sample_count = imu_sample_count;
}

retransmit_rx_state = 0;

retransmit_rx_start_status =
    HAL_UART_Receive_IT(
        &huart1,
        &retransmit_sync_byte,
        1
    );
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
     if (imu_sample_flag)
    {
        /* 先清flag */
        imu_sample_flag = 0;


        /* 读取MPU6050 */
        imu_read_status = MPU6050_ReadRaw(&imu_raw);


        if (imu_read_status == HAL_OK)
        {
            /* RAW -> g / deg/s */
            MPU6050_ConvertRaw(
                &imu_raw,
                &imu_data
            );


            /* 陀螺仪零偏校正 */
            MPU6050_ApplyGyroBias(
                &imu_data,
                &gyro_bias
            );
					  
					  Mahony_UpdateIMU(
               &mahony,

               imu_data.gx,
               imu_data.gy,
               imu_data.gz,

               imu_data.ax,
               imu_data.ay,
               imu_data.az,

               0.001f
            );
            /* 一次完整采样成功 */
            imu_sample_count++;
            
						/* Copy current attitude */
attitude_tx_data.q0 =
    mahony.q0;

attitude_tx_data.q1 =
    mahony.q1;

attitude_tx_data.q2 =
    mahony.q2;

attitude_tx_data.q3 =
    mahony.q3;

attitude_tx_data.roll =
    mahony.roll;

attitude_tx_data.pitch =
    mahony.pitch;

attitude_tx_data.yaw =
    mahony.yaw;


/* Build protocol frame */
Protocol_BuildAttitudeFrame(
    attitude_tx_frame,
    attitude_tx_sequence,
    &attitude_tx_data
);


/*
 * Save the correct frame before sending.
 *
 * If master later requests this sequence,
 * the original correct frame can be resent.
 */
memcpy(
    attitude_history[
        attitude_history_write_index
    ],
    attitude_tx_frame,
    PROTOCOL_ATTITUDE_FRAME_LEN
);

attitude_history_sequence[
    attitude_history_write_index
] = attitude_tx_sequence;

attitude_history_valid[
    attitude_history_write_index
] = 1;


attitude_history_write_index++;

if (attitude_history_write_index >=
    ATTITUDE_HISTORY_SIZE)
{
    attitude_history_write_index = 0;
}


/*
 * Test only:
 *
 * Intentionally corrupt one payload byte
 * when sequence reaches 3000.
 *
 * CRC remains unchanged, therefore master
 * must detect PROTOCOL_ERROR_CRC.
 *
 * The history copy above remains correct.
 */
if ((protocol_test_enable != 0) &&
    (protocol_test_done == 0) &&
    (attitude_tx_sequence ==
     PROTOCOL_TEST_CORRUPT_SEQUENCE))
{
    attitude_tx_frame[10] ^= 0x01;

    protocol_test_done = 1;
}


if (gains_ack_pending != 0)
{
    gains_ack_tx_status =
        HAL_UART_Transmit(
            &huart1,
            gains_ack_frame,
            PROTOCOL_GAINS_ACK_FRAME_LEN,
            2
        );


    if (gains_ack_tx_status == HAL_OK)
    {
        gains_ack_pending = 0;

        gains_ack_tx_count++;
    }
    else
    {
        gains_ack_tx_error_count++;
    }
}
else
{
    /* Send normal attitude frame */
    attitude_tx_status =
        HAL_UART_Transmit(
            &huart1,
            attitude_tx_frame,
            PROTOCOL_ATTITUDE_FRAME_LEN,
            2
        );


    if (attitude_tx_status == HAL_OK)
    {
        attitude_tx_count++;

        attitude_tx_sequence++;
    }
    else
    {
        attitude_tx_error_count++;
    }
}
        }
				
    }
		
		

/*
 * Process retransmit request
 */
if (retransmit_request_pending)
{
    if (retransmit_rx_frame[2] ==
        PROTOCOL_CMD_RETRANSMIT)
    {
        retransmit_request_status =
            Protocol_ParseRetransmitRequest(
                retransmit_rx_frame,
                PROTOCOL_RETRANSMIT_REQUEST_LEN,
                (uint16_t *)
                &retransmit_requested_sequence
            );


        if (retransmit_request_status ==
            PROTOCOL_OK)
        {
            retransmit_request_count++;


            if (Slave_FindHistoryFrame(
                    retransmit_requested_sequence,
                    retransmit_tx_frame))
            {
                retransmit_tx_pending = 1;
            }
            else
            {
                retransmit_not_found_count++;
            }
        }
    }
    else if (retransmit_rx_frame[2] ==
             PROTOCOL_CMD_SET_GAINS)
    {
        set_gains_status =
            Protocol_ParseSetGainsFrame(
                retransmit_rx_frame,
                retransmit_rx_frame_length,
                &set_gains_sequence,
                &set_gains_kp,
                &set_gains_ki
            );


        if (set_gains_status ==
            PROTOCOL_OK)
        {
            Mahony_SetGains(
                &mahony,
                set_gains_kp,
                set_gains_ki
            );


            Protocol_BuildGainsAckFrame(
                gains_ack_frame,
                set_gains_sequence,
                mahony.kp,
                mahony.ki,
                PROTOCOL_GAINS_ACK_OK
            );


            gains_ack_pending = 1;

            set_gains_count++;
        }
        else
        {
            set_gains_error_count++;
        }
    }
    else
    {
        set_gains_error_count++;
    }


    retransmit_request_pending = 0;

    retransmit_rx_state = 0;

    retransmit_rx_frame_length = 0;


    /*
     * Wait for next retransmit request.
     */
    HAL_UART_Receive_IT(
        &huart1,
        &retransmit_sync_byte,
        1
    );
}


/*
 * Send retransmitted frame asynchronously.
 *
 * Using interrupt TX here avoids blocking
 * the 1 kHz IMU task for another 36 bytes.
 */
if ((retransmit_tx_pending != 0) &&
    (retransmit_tx_in_progress == 0))
{
    retransmit_tx_pending = 0;


    retransmit_tx_status =
        HAL_UART_Transmit_IT(
            &huart1,
            retransmit_tx_frame,
            PROTOCOL_ATTITUDE_FRAME_LEN
        );


    if (retransmit_tx_status == HAL_OK)
    {
        retransmit_tx_in_progress = 1;
    }
    else
    {
        retransmit_tx_error_count++;
    }
}
		/* ==================== 每1秒统计一次采样率 ==================== */
    if ((HAL_GetTick() - rate_last_tick) >= 1000)
    {
        imu_sample_rate =
            imu_sample_count -
            rate_last_sample_count;

        rate_last_sample_count =
            imu_sample_count;

        rate_last_tick += 1000;
    }
  /* USER CODE END 3 */
}

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        imu_timer_tick_count++;

        if (imu_sample_flag == 0)
        {
            imu_sample_flag = 1;
        }
        else
        {
            /*
             * 上一次1ms任务还没处理，
             * 下一次中断已经来了。
             */
            imu_overrun_count++;
        }
    }
}

void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef *huart
)
{
    if (huart->Instance == USART1)
    {
        /*
         * State 0:
         * wait for 0xAA
         */
        if (retransmit_rx_state == 0)
        {
            if (retransmit_sync_byte ==
                PROTOCOL_HEADER_1)
            {
                retransmit_rx_frame[0] =
                    retransmit_sync_byte;

                retransmit_rx_state = 1;
            }


            HAL_UART_Receive_IT(
                &huart1,
                &retransmit_sync_byte,
                1
            );
        }


        /*
         * State 1:
         * wait for 0x55
         */
        else if (retransmit_rx_state == 1)
        {
            if (retransmit_sync_byte ==
                PROTOCOL_HEADER_2)
            {
                retransmit_rx_frame[1] =
                    retransmit_sync_byte;

                retransmit_rx_state = 2;


                /*
                 * Receive CMD and LEN.
                 */
                HAL_UART_Receive_IT(
                    &huart1,
                    &retransmit_rx_frame[2],
                    2
                );
            }
            else
            {
                if (retransmit_sync_byte ==
                    PROTOCOL_HEADER_1)
                {
                    retransmit_rx_frame[0] =
                        retransmit_sync_byte;

                    retransmit_rx_state = 1;
                }
                else
                {
                    retransmit_rx_state = 0;
                }


                HAL_UART_Receive_IT(
                    &huart1,
                    &retransmit_sync_byte,
                    1
                );
            }
        }


        /*
         * State 2:
         * CMD and LEN received
         */
        else if (retransmit_rx_state == 2)
        {
            if ((retransmit_rx_frame[2] ==
                 PROTOCOL_CMD_RETRANSMIT) &&
                (retransmit_rx_frame[3] == 0))
            {
                retransmit_rx_frame_length =
                    PROTOCOL_RETRANSMIT_REQUEST_LEN;

                retransmit_rx_state = 3;


                HAL_UART_Receive_IT(
                    &huart1,
                    &retransmit_rx_frame[4],
                    PROTOCOL_RETRANSMIT_REQUEST_LEN - 4
                );
            }
            else if (
                (retransmit_rx_frame[2] ==
                 PROTOCOL_CMD_SET_GAINS) &&
                (retransmit_rx_frame[3] ==
                 PROTOCOL_GAINS_PAYLOAD_LEN))
            {
                retransmit_rx_frame_length =
                    PROTOCOL_SET_GAINS_FRAME_LEN;

                retransmit_rx_state = 3;


                HAL_UART_Receive_IT(
                    &huart1,
                    &retransmit_rx_frame[4],
                    PROTOCOL_SET_GAINS_FRAME_LEN - 4
                );
            }
            else
            {
                retransmit_rx_frame_length = 0;

                retransmit_rx_state = 0;


                HAL_UART_Receive_IT(
                    &huart1,
                    &retransmit_sync_byte,
                    1
                );
            }
        }


        /*
         * State 3:
         * complete command received
         */
        else if (retransmit_rx_state == 3)
        {
            retransmit_request_pending = 1;

            retransmit_rx_state = 4;
        }
    }
}

void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef *huart
)
{
    if (huart->Instance == USART1)
    {
        if (retransmit_tx_in_progress != 0)
        {
            retransmit_tx_in_progress = 0;

            retransmit_tx_count++;
        }
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
  * @brief  Reports the name of the source file name and the source line number
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