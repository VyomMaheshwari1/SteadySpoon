/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SteadyHand prototype - MPU6050 + complementary filter +
  *                   one DS215 servo PWM test on TIM4 CH1 / PB6
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define MPU6050_ADDR             (0x68 << 1)
#define MPU6050_WHO_AM_I         0x75
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_GYRO_CONFIG      0x1B
#define MPU6050_ACCEL_CONFIG     0x1C
#define MPU6050_ACCEL_XOUT_H     0x3B

#define SERVO_CENTER_US          1500U
#define SERVO_TEST_LEFT_US       1400U
#define SERVO_TEST_RIGHT_US      1600U
#define SERVO_MIN_US             1000U
#define SERVO_MAX_US             2000U
/* USER CODE END PD */

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint8_t who_am_i = 0;

volatile HAL_StatusTypeDef imu_status;
volatile HAL_StatusTypeDef wake_status;
volatile HAL_StatusTypeDef gyro_config_status;
volatile HAL_StatusTypeDef accel_config_status;
volatile HAL_StatusTypeDef data_status;
volatile HAL_StatusTypeDef servo_pwm_status;

uint8_t sensor_data[14];

volatile int16_t accel_x_raw;
volatile int16_t accel_y_raw;
volatile int16_t accel_z_raw;
volatile int16_t gyro_x_raw;
volatile int16_t gyro_y_raw;
volatile int16_t gyro_z_raw;

volatile float accel_x_g;
volatile float accel_y_g;
volatile float accel_z_g;

volatile float gyro_x_dps;
volatile float gyro_y_dps;
volatile float gyro_z_dps;

volatile float gyro_x_bias = 0.0f;
volatile float gyro_y_bias = 0.0f;
volatile float gyro_z_bias = 0.0f;

volatile float accel_tilt_deg = 0.0f;
volatile float filtered_tilt_deg = 0.0f;
volatile uint8_t filter_initialized = 0;

volatile uint16_t servo_pulse_us = SERVO_CENTER_US;
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
static void Servo_SetPulse(uint16_t pulse_us);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void Servo_SetPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_US)
    {
        pulse_us = SERVO_MIN_US;
    }
    else if (pulse_us > SERVO_MAX_US)
    {
        pulse_us = SERVO_MAX_US;
    }

    servo_pulse_us = pulse_us;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  imu_status = HAL_I2C_Mem_Read(
      &hi2c1,
      MPU6050_ADDR,
      MPU6050_WHO_AM_I,
      I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&who_am_i,
      1,
      100
  );

  uint8_t wake = 0x00;

  wake_status = HAL_I2C_Mem_Write(
      &hi2c1,
      MPU6050_ADDR,
      MPU6050_PWR_MGMT_1,
      I2C_MEMADD_SIZE_8BIT,
      &wake,
      1,
      100
  );

  HAL_Delay(100);

  uint8_t gyro_config = 0x08;

  gyro_config_status = HAL_I2C_Mem_Write(
      &hi2c1,
      MPU6050_ADDR,
      MPU6050_GYRO_CONFIG,
      I2C_MEMADD_SIZE_8BIT,
      &gyro_config,
      1,
      100
  );

  uint8_t accel_config = 0x00;

  accel_config_status = HAL_I2C_Mem_Write(
      &hi2c1,
      MPU6050_ADDR,
      MPU6050_ACCEL_CONFIG,
      I2C_MEMADD_SIZE_8BIT,
      &accel_config,
      1,
      100
  );

  HAL_Delay(100);

  /* Keep MPU6050 still during this ~1 second gyro calibration. */
  gyro_x_bias = 0.0f;
  gyro_y_bias = 0.0f;
  gyro_z_bias = 0.0f;

  uint16_t calibration_samples = 0;

  for (int i = 0; i < 200; i++)
  {
      HAL_StatusTypeDef calibration_status;

      calibration_status = HAL_I2C_Mem_Read(
          &hi2c1,
          MPU6050_ADDR,
          MPU6050_ACCEL_XOUT_H,
          I2C_MEMADD_SIZE_8BIT,
          sensor_data,
          14,
          100
      );

      if (calibration_status == HAL_OK)
      {
          int16_t gx =
              (int16_t)((sensor_data[8] << 8) | sensor_data[9]);

          int16_t gy =
              (int16_t)((sensor_data[10] << 8) | sensor_data[11]);

          int16_t gz =
              (int16_t)((sensor_data[12] << 8) | sensor_data[13]);

          gyro_x_bias += gx / 65.5f;
          gyro_y_bias += gy / 65.5f;
          gyro_z_bias += gz / 65.5f;
          calibration_samples++;
      }

      HAL_Delay(5);
  }

  if (calibration_samples > 0)
  {
      gyro_x_bias /= calibration_samples;
      gyro_y_bias /= calibration_samples;
      gyro_z_bias /= calibration_samples;
  }

  /* Start PWM for one DS215 servo on TIM4 CH1 / PB6 (D6). */
  Servo_SetPulse(SERVO_CENTER_US);
  servo_pwm_status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  /* One-time safe servo test: center -> small left -> small right -> center. */
  if (servo_pwm_status == HAL_OK)
  {
      Servo_SetPulse(SERVO_CENTER_US);
      HAL_Delay(800);

      Servo_SetPulse(SERVO_TEST_LEFT_US);
      HAL_Delay(800);

      Servo_SetPulse(SERVO_TEST_RIGHT_US);
      HAL_Delay(800);

      Servo_SetPulse(SERVO_CENTER_US);
      HAL_Delay(800);
  }

  /* USER CODE END 2 */

  while (1)
  {
      data_status = HAL_I2C_Mem_Read(
          &hi2c1,
          MPU6050_ADDR,
          MPU6050_ACCEL_XOUT_H,
          I2C_MEMADD_SIZE_8BIT,
          sensor_data,
          14,
          100
      );

      if (data_status == HAL_OK)
      {
          accel_x_raw =
              (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
          accel_y_raw =
              (int16_t)((sensor_data[2] << 8) | sensor_data[3]);
          accel_z_raw =
              (int16_t)((sensor_data[4] << 8) | sensor_data[5]);

          gyro_x_raw =
              (int16_t)((sensor_data[8] << 8) | sensor_data[9]);
          gyro_y_raw =
              (int16_t)((sensor_data[10] << 8) | sensor_data[11]);
          gyro_z_raw =
              (int16_t)((sensor_data[12] << 8) | sensor_data[13]);

          accel_x_g = accel_x_raw / 16384.0f;
          accel_y_g = accel_y_raw / 16384.0f;
          accel_z_g = accel_z_raw / 16384.0f;

          gyro_x_dps = (gyro_x_raw / 65.5f) - gyro_x_bias;
          gyro_y_dps = (gyro_y_raw / 65.5f) - gyro_y_bias;
          gyro_z_dps = (gyro_z_raw / 65.5f) - gyro_z_bias;

          accel_tilt_deg =
              atan2f(accel_x_g, -accel_z_g) * 57.29578f;

          if (filter_initialized == 0)
          {
              filtered_tilt_deg = accel_tilt_deg;
              filter_initialized = 1;
          }
          else
          {
              const float dt = 0.010f;
              const float alpha = 0.98f;

              filtered_tilt_deg =
                  alpha * (filtered_tilt_deg + gyro_y_dps * dt)
                  + (1.0f - alpha) * accel_tilt_deg;
          }
      }

      HAL_Delay(10);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(
          &RCC_ClkInitStruct,
          FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(
          &hi2c1,
          I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(
          &htim1,
          &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(
          &htim1,
          &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3002;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(
          &htim4,
          &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(
          &htim4,
          &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(
          &htim4,
          &sConfigOC,
          TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
