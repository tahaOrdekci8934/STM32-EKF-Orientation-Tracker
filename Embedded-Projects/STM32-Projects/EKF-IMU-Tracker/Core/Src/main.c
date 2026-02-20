
/* Includes ------------------------------------------------------------------*/
#include <EKF_algorithm.h>
#include <mpu6050_driver.h>
#include <telemetry.h>
#include <timing.h>
#include "main.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef  hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
EKF_Handle        h_ekf;
TelemetryPacket_t tx_packet;

/* IMU raw data */
float ax, ay, az;
float gx, gy, gz;

/* Gyroscope Z-axis bias compensation */
float gz_offset = 0.005f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset peripherals, initialize Flash interface and SysTick */
  HAL_Init();

  /* Configure system clock */
  SystemClock_Config();

  /* Initialize configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* 1. Initialize register-level timer (50 ms periodic interrupt) */
  Timing_Init();

  /* 2. Initialize register-level DMA for telemetry transmission */
  Telemetry_DMA_Init();

  /* 3. Initialize MPU6050 IMU */
  if (MPU6050_Init(I2C1) != 0)
  {
      /* IMU initialization failed:
         Turn LED on and halt execution for fault indication */
      GPIOA->BSRR = (1 << 5);
      while (1);
  }

  /* Turn LED off after successful initialization */
  GPIOA->BRR = (1 << 5);

  /* 4. Initialize Extended Kalman Filter (dt = 50 ms) */
  EKF_Init(&h_ekf, 0.05f);

  /* 5. Set constant telemetry sync word */
  tx_packet.sync_word = 0x7F7F7F7F;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* Execute EKF and telemetry pipeline on timer trigger */
      if (ekf_trigger_flag)
      {
          ekf_trigger_flag = 0;  /* Clear trigger flag */

          /* Step A: Read IMU data via register-level I2C burst */
          MPU6050_Read_Raw_Data(I2C1, &ax, &ay, &az, &gx, &gy, &gz);

          /* Step B: Gyroscope bias compensation and deg/s → rad/s conversion */
          float gx_rad = gx * 0.01745329f;
          float gy_rad = gy * 0.01745329f;
          float gz_rad = (gz - gz_offset) * 0.01745329f;

          /* Normalize accelerometer measurement */
          float a_norm = sqrtf(ax * ax + ay * ay + az * az);
          if (a_norm > 0.0001f)
          {
              ax /= a_norm;
              ay /= a_norm;
              az /= a_norm;
          }

          /* Step C: EKF prediction and correction */
          EKF_Predict(&h_ekf, gx_rad, gy_rad, gz_rad);
          EKF_Update(&h_ekf, ax, ay, az);

          /* Step D: Package telemetry data */
          for (int i = 0; i < 4; i++)
          {
              tx_packet.q[i] = h_ekf.x_data[i];
          }

          tx_packet.accel[0] = ax;
          tx_packet.accel[1] = ay;
          tx_packet.accel[2] = az;

          /* Step E: Non-blocking DMA telemetry transmission */
          Telemetry_Send_Burst(&tx_packet, sizeof(tx_packet));

          /* Toggle status LED */
          GPIOA->ODR ^= (1 << 5);
      }
  }
  /* USER CODE END WHILE */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
      Error_Handler();
  }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  |
                                    RCC_CLOCKTYPE_SYSCLK|
                                    RCC_CLOCKTYPE_PCLK1 |
                                    RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* Enable AFIO and GPIOB clocks */
  RCC->APB2ENR |= (1 << 0) | (1 << 3);

  /* Remap I2C1 to PB8 (SCL) and PB9 (SDA) */
  AFIO->MAPR |= (1 << 1);

  /* Configure PB8 and PB9 as Alternate Function Open-Drain, 50 MHz */
  GPIOB->CRH &= ~(0xFF << 0);
  GPIOB->CRH |=  (0xEE << 0);

  /* USER CODE END I2C1_Init 0 */

  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
      Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

