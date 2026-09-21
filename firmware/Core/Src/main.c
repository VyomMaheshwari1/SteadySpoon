#include "main.h"
#include "cmsis_os2.h"
#include <math.h>

/* ============================================================
 * MPU6050 DEFINITIONS
 * ============================================================ */

#define MPU6050_ADDR              (0x68 << 1)
#define MPU6050_WHO_AM_I          0x75
#define MPU6050_PWR_MGMT_1        0x6B
#define MPU6050_GYRO_CONFIG       0x1B
#define MPU6050_ACCEL_CONFIG      0x1C
#define MPU6050_ACCEL_XOUT_H      0x3B


/* ============================================================
 * SERVO 1
 * ============================================================ */

#define SERVO1_MIN_US             1000U
#define SERVO1_CENTER_US          1500U
#define SERVO1_MAX_US             2000U

#define SERVO1_GAIN               12.0f
#define SERVO1_DIRECTION          (-1.0f)


/* ============================================================
 * SERVO 2
 * ============================================================ */

#define SERVO2_MIN_US             1000U
#define SERVO2_CENTER_US          1500U
#define SERVO2_MAX_US             2000U

#define SERVO2_GAIN               12.0f

/*
 * If Servo2 compensates in the wrong direction,
 * later change this between -1.0f and +1.0f.
 */
#define SERVO2_DIRECTION          (-1.0f)


#define ANGLE_DEADBAND_DEG        0.3f


/* ============================================================
 * STM32 PERIPHERAL HANDLES
 * ============================================================ */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;


/* ============================================================
 * MPU6050 STRUCTURE
 * ============================================================ */

typedef struct
{
    I2C_HandleTypeDef *i2c;

    uint8_t who_am_i;
    uint8_t ok;
    uint8_t filter_initialized;

    uint8_t sensor_data[14];

    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;

    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float gyro_x_bias;
    float gyro_y_bias;
    float gyro_z_bias;

    float pitch_accel_deg;
    float roll_accel_deg;

    float pitch_filtered_deg;
    float roll_filtered_deg;

    float pitch_reference_deg;
    float roll_reference_deg;

    uint32_t last_update_ms;

} MPU6050_t;


/* ============================================================
 * MPU OBJECTS
 * ============================================================ */

MPU6050_t mpu1 = {0};
MPU6050_t mpu2 = {0};


/* ============================================================
 * SERVO / DEBUG VARIABLES
 * ============================================================ */

volatile uint16_t servo1_pulse_us =
    SERVO1_CENTER_US;

volatile uint16_t servo2_pulse_us =
    SERVO2_CENTER_US;


volatile float servo1_relative_angle = 0.0f;
volatile float servo1_error = 0.0f;
volatile float servo1_correction = 0.0f;

volatile float servo2_relative_angle = 0.0f;
volatile float servo2_error = 0.0f;
volatile float servo2_correction = 0.0f;


HAL_StatusTypeDef servo1_pwm_status;
HAL_StatusTypeDef servo2_pwm_status;


/*
 * These let us verify FreeRTOS is actually running.
 *
 * imu_task_count should increase about twice
 * as fast as control_task_count.
 */
volatile uint32_t imu_task_count = 0;
volatile uint32_t control_task_count = 0;


/* ============================================================
 * FUNCTION PROTOTYPES
 * ============================================================ */

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);


/*
 * Generated/configured by the MSP file.
 */
void HAL_TIM_MspPostInit(
    TIM_HandleTypeDef *htim
);


/*
 * Implemented in app_freertos.c.
 */
extern void MX_FREERTOS_Init(void);


/* MPU functions */

static void MPU6050_Init(
    MPU6050_t *mpu
);

static void MPU6050_CalibrateGyro(
    MPU6050_t *mpu
);

static void MPU6050_Update(
    MPU6050_t *mpu
);

static void MPU6050_CalibrateReference(
    MPU6050_t *mpu
);


/* Servo functions */

static void Servo1_SetPulse(
    uint16_t pulse_us
);

static void Servo2_SetPulse(
    uint16_t pulse_us
);

static void Servo1_Stabilization_Update(void);
static void Servo2_Stabilization_Update(void);


/*
 * These are called by app_freertos.c.
 */
void App_IMU_TaskStep(void);
void App_Control_TaskStep(void);


/* ============================================================
 * SERVO 1 PWM
 * ============================================================ */

static void Servo1_SetPulse(
    uint16_t pulse_us
)
{
    if (pulse_us < SERVO1_MIN_US)
    {
        pulse_us = SERVO1_MIN_US;
    }

    if (pulse_us > SERVO1_MAX_US)
    {
        pulse_us = SERVO1_MAX_US;
    }

    servo1_pulse_us =
        pulse_us;

    __HAL_TIM_SET_COMPARE(
        &htim4,
        TIM_CHANNEL_1,
        pulse_us
    );
}


/* ============================================================
 * SERVO 2 PWM
 * ============================================================ */

static void Servo2_SetPulse(
    uint16_t pulse_us
)
{
    if (pulse_us < SERVO2_MIN_US)
    {
        pulse_us = SERVO2_MIN_US;
    }

    if (pulse_us > SERVO2_MAX_US)
    {
        pulse_us = SERVO2_MAX_US;
    }

    servo2_pulse_us =
        pulse_us;

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_3,
        pulse_us
    );
}


/* ============================================================
 * SERVO 1 STABILIZATION
 *
 * MPU1 pitch -> Servo1
 * ============================================================ */

static void Servo1_Stabilization_Update(void)
{
    if (
        !mpu1.ok ||
        !mpu1.filter_initialized
    )
    {
        return;
    }


    servo1_relative_angle =
        mpu1.pitch_filtered_deg -
        mpu1.pitch_reference_deg;


    servo1_error =
        servo1_relative_angle;


    if (
        fabsf(servo1_error) <
        ANGLE_DEADBAND_DEG
    )
    {
        servo1_error = 0.0f;
    }


    servo1_correction =
        servo1_error *
        SERVO1_GAIN;


    float command =
        (float)SERVO1_CENTER_US +
        (
            SERVO1_DIRECTION *
            servo1_correction
        );


    int32_t servo_command =
        (int32_t)command;


    if (
        servo_command <
        (int32_t)SERVO1_MIN_US
    )
    {
        servo_command =
            SERVO1_MIN_US;
    }


    if (
        servo_command >
        (int32_t)SERVO1_MAX_US
    )
    {
        servo_command =
            SERVO1_MAX_US;
    }


    Servo1_SetPulse(
        (uint16_t)servo_command
    );
}


/* ============================================================
 * SERVO 2 STABILIZATION
 *
 * MPU2 roll -> Servo2
 * ============================================================ */

static void Servo2_Stabilization_Update(void)
{
    if (
        !mpu2.ok ||
        !mpu2.filter_initialized
    )
    {
        return;
    }


    servo2_relative_angle =
        mpu2.roll_filtered_deg -
        mpu2.roll_reference_deg;


    servo2_error =
        servo2_relative_angle;


    if (
        fabsf(servo2_error) <
        ANGLE_DEADBAND_DEG
    )
    {
        servo2_error = 0.0f;
    }


    servo2_correction =
        servo2_error *
        SERVO2_GAIN;


    float command =
        (float)SERVO2_CENTER_US +
        (
            SERVO2_DIRECTION *
            servo2_correction
        );


    int32_t servo_command =
        (int32_t)command;


    if (
        servo_command <
        (int32_t)SERVO2_MIN_US
    )
    {
        servo_command =
            SERVO2_MIN_US;
    }


    if (
        servo_command >
        (int32_t)SERVO2_MAX_US
    )
    {
        servo_command =
            SERVO2_MAX_US;
    }


    Servo2_SetPulse(
        (uint16_t)servo_command
    );
}


/* ============================================================
 * MPU6050 INITIALIZATION
 * ============================================================ */

static void MPU6050_Init(
    MPU6050_t *mpu
)
{
    uint8_t wake =
        0x00;

    /*
     * Gyroscope +/-500 degrees/sec.
     */
    uint8_t gyro_config =
        0x08;

    /*
     * Accelerometer +/-2g.
     */
    uint8_t accel_config =
        0x00;


    HAL_StatusTypeDef status;


    /*
     * Ask the MPU who it is.
     */
    status =
        HAL_I2C_Mem_Read(
            mpu->i2c,
            MPU6050_ADDR,
            MPU6050_WHO_AM_I,
            I2C_MEMADD_SIZE_8BIT,
            &mpu->who_am_i,
            1,
            100
        );


    /*
     * MPU6050 should answer 0x68.
     */
    if (
        status == HAL_OK &&
        mpu->who_am_i == 0x68
    )
    {
        mpu->ok = 1;
    }
    else
    {
        mpu->ok = 0;
        return;
    }


    /*
     * Wake the MPU6050.
     */
    HAL_I2C_Mem_Write(
        mpu->i2c,
        MPU6050_ADDR,
        MPU6050_PWR_MGMT_1,
        I2C_MEMADD_SIZE_8BIT,
        &wake,
        1,
        100
    );


    HAL_Delay(100);


    /*
     * Configure gyroscope.
     */
    HAL_I2C_Mem_Write(
        mpu->i2c,
        MPU6050_ADDR,
        MPU6050_GYRO_CONFIG,
        I2C_MEMADD_SIZE_8BIT,
        &gyro_config,
        1,
        100
    );


    /*
     * Configure accelerometer.
     */
    HAL_I2C_Mem_Write(
        mpu->i2c,
        MPU6050_ADDR,
        MPU6050_ACCEL_CONFIG,
        I2C_MEMADD_SIZE_8BIT,
        &accel_config,
        1,
        100
    );


    HAL_Delay(100);
}


/* ============================================================
 * GYROSCOPE CALIBRATION
 * ============================================================ */

static void MPU6050_CalibrateGyro(
    MPU6050_t *mpu
)
{
    if (!mpu->ok)
    {
        return;
    }


    mpu->gyro_x_bias = 0.0f;
    mpu->gyro_y_bias = 0.0f;
    mpu->gyro_z_bias = 0.0f;


    uint16_t successful_samples = 0;


    /*
     * Take 200 samples while sensor is stationary.
     */
    for (
        int i = 0;
        i < 200;
        i++
    )
    {
        HAL_StatusTypeDef status;


        status =
            HAL_I2C_Mem_Read(
                mpu->i2c,
                MPU6050_ADDR,
                MPU6050_ACCEL_XOUT_H,
                I2C_MEMADD_SIZE_8BIT,
                mpu->sensor_data,
                14,
                100
            );


        if (status == HAL_OK)
        {
            int16_t gx =
                (int16_t)(
                    (mpu->sensor_data[8] << 8) |
                    mpu->sensor_data[9]
                );


            int16_t gy =
                (int16_t)(
                    (mpu->sensor_data[10] << 8) |
                    mpu->sensor_data[11]
                );


            int16_t gz =
                (int16_t)(
                    (mpu->sensor_data[12] << 8) |
                    mpu->sensor_data[13]
                );


            mpu->gyro_x_bias +=
                gx / 65.5f;


            mpu->gyro_y_bias +=
                gy / 65.5f;


            mpu->gyro_z_bias +=
                gz / 65.5f;


            successful_samples++;
        }


        HAL_Delay(5);
    }


    if (successful_samples > 0)
    {
        mpu->gyro_x_bias /=
            successful_samples;

        mpu->gyro_y_bias /=
            successful_samples;

        mpu->gyro_z_bias /=
            successful_samples;
    }
}


/* ============================================================
 * READ MPU + COMPLEMENTARY FILTER
 * ============================================================ */

static void MPU6050_Update(
    MPU6050_t *mpu
)
{
    if (!mpu->ok)
    {
        return;
    }


    HAL_StatusTypeDef status;


    status =
        HAL_I2C_Mem_Read(
            mpu->i2c,
            MPU6050_ADDR,
            MPU6050_ACCEL_XOUT_H,
            I2C_MEMADD_SIZE_8BIT,
            mpu->sensor_data,
            14,
            100
        );


    if (status != HAL_OK)
    {
        return;
    }


    /* Accelerometer */

    mpu->accel_x_raw =
        (int16_t)(
            (mpu->sensor_data[0] << 8) |
            mpu->sensor_data[1]
        );


    mpu->accel_y_raw =
        (int16_t)(
            (mpu->sensor_data[2] << 8) |
            mpu->sensor_data[3]
        );


    mpu->accel_z_raw =
        (int16_t)(
            (mpu->sensor_data[4] << 8) |
            mpu->sensor_data[5]
        );


    /* Gyroscope */

    mpu->gyro_x_raw =
        (int16_t)(
            (mpu->sensor_data[8] << 8) |
            mpu->sensor_data[9]
        );


    mpu->gyro_y_raw =
        (int16_t)(
            (mpu->sensor_data[10] << 8) |
            mpu->sensor_data[11]
        );


    mpu->gyro_z_raw =
        (int16_t)(
            (mpu->sensor_data[12] << 8) |
            mpu->sensor_data[13]
        );


    /* Convert accelerometer to g */

    mpu->accel_x_g =
        mpu->accel_x_raw /
        16384.0f;


    mpu->accel_y_g =
        mpu->accel_y_raw /
        16384.0f;


    mpu->accel_z_g =
        mpu->accel_z_raw /
        16384.0f;


    /* Convert gyro to degrees per second */

    mpu->gyro_x_dps =
        (
            mpu->gyro_x_raw /
            65.5f
        )
        -
        mpu->gyro_x_bias;


    mpu->gyro_y_dps =
        (
            mpu->gyro_y_raw /
            65.5f
        )
        -
        mpu->gyro_y_bias;


    mpu->gyro_z_dps =
        (
            mpu->gyro_z_raw /
            65.5f
        )
        -
        mpu->gyro_z_bias;


    /* Pitch from accelerometer */

    mpu->pitch_accel_deg =
        atan2f(
            -mpu->accel_x_g,

            sqrtf(
                (
                    mpu->accel_y_g *
                    mpu->accel_y_g
                )
                +
                (
                    mpu->accel_z_g *
                    mpu->accel_z_g
                )
            )
        )
        *
        57.29578f;


    /* Roll from accelerometer */

    mpu->roll_accel_deg =
        atan2f(
            mpu->accel_y_g,

            sqrtf(
                (
                    mpu->accel_x_g *
                    mpu->accel_x_g
                )
                +
                (
                    mpu->accel_z_g *
                    mpu->accel_z_g
                )
            )
        )
        *
        57.29578f;


    /*
     * Measure elapsed time.
     */
    uint32_t now =
        HAL_GetTick();


    float dt =
        0.005f;


    if (
        mpu->last_update_ms != 0
    )
    {
        dt =
            (
                now -
                mpu->last_update_ms
            )
            *
            0.001f;


        if (
            dt <= 0.0f ||
            dt > 0.05f
        )
        {
            dt =
                0.005f;
        }
    }


    mpu->last_update_ms =
        now;


    /*
     * Complementary filter.
     *
     * 98 percent gyro
     * 2 percent accelerometer
     */
    const float alpha =
        0.98f;


    if (
        !mpu->filter_initialized
    )
    {
        mpu->pitch_filtered_deg =
            mpu->pitch_accel_deg;


        mpu->roll_filtered_deg =
            mpu->roll_accel_deg;


        mpu->filter_initialized =
            1;
    }
    else
    {
        mpu->pitch_filtered_deg =
            alpha *
            (
                mpu->pitch_filtered_deg +
                (
                    mpu->gyro_y_dps *
                    dt
                )
            )
            +
            (
                1.0f -
                alpha
            )
            *
            mpu->pitch_accel_deg;


        mpu->roll_filtered_deg =
            alpha *
            (
                mpu->roll_filtered_deg +
                (
                    mpu->gyro_x_dps *
                    dt
                )
            )
            +
            (
                1.0f -
                alpha
            )
            *
            mpu->roll_accel_deg;
    }
}


/* ============================================================
 * REFERENCE ANGLE CALIBRATION
 * ============================================================ */

static void MPU6050_CalibrateReference(
    MPU6050_t *mpu
)
{
    if (!mpu->ok)
    {
        return;
    }


    /*
     * Let filter settle.
     */
    for (
        int i = 0;
        i < 100;
        i++
    )
    {
        MPU6050_Update(
            mpu
        );

        HAL_Delay(5);
    }


    float pitch_sum =
        0.0f;

    float roll_sum =
        0.0f;

    uint16_t samples =
        0;


    /*
     * Average 100 samples.
     */
    for (
        int i = 0;
        i < 100;
        i++
    )
    {
        MPU6050_Update(
            mpu
        );


        if (
            mpu->filter_initialized
        )
        {
            pitch_sum +=
                mpu->pitch_filtered_deg;


            roll_sum +=
                mpu->roll_filtered_deg;


            samples++;
        }


        HAL_Delay(5);
    }


    if (samples > 0)
    {
        mpu->pitch_reference_deg =
            pitch_sum /
            samples;


        mpu->roll_reference_deg =
            roll_sum /
            samples;
    }
}


/* ============================================================
 * FREERTOS IMU TASK STEP
 *
 * app_freertos.c calls this at 200 Hz.
 * ============================================================ */

void App_IMU_TaskStep(void)
{
    MPU6050_Update(
        &mpu1
    );


    MPU6050_Update(
        &mpu2
    );


    imu_task_count++;
}


/* ============================================================
 * FREERTOS CONTROL TASK STEP
 *
 * app_freertos.c calls this at 100 Hz.
 * ============================================================ */

void App_Control_TaskStep(void)
{
    if (
        servo1_pwm_status ==
        HAL_OK
    )
    {
        Servo1_Stabilization_Update();
    }


    if (
        servo2_pwm_status ==
        HAL_OK
    )
    {
        Servo2_Stabilization_Update();
    }


    control_task_count++;
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    /*
     * Initialize STM32 HAL.
     *
     * HAL timing now uses TIM6 because
     * FreeRTOS uses SysTick.
     */
    HAL_Init();


    /*
     * Configure 16 MHz HSI clock.
     */
    SystemClock_Config();


    /*
     * Initialize GPIO.
     */
    MX_GPIO_Init();


    /*
     * Initialize both I2C peripherals.
     */
    MX_I2C1_Init();
    MX_I2C2_Init();


    /*
     * Initialize both servo timers.
     */
    MX_TIM3_Init();
    MX_TIM4_Init();


    /*
     * Attach MPU1 to I2C1.
     */
    mpu1.i2c =
        &hi2c1;


    /*
     * Attach MPU2 to I2C2.
     */
    mpu2.i2c =
        &hi2c2;


    /*
     * Initialize sensors.
     */
    MPU6050_Init(
        &mpu1
    );


    MPU6050_Init(
        &mpu2
    );


    /*
     * KEEP BOTH MPUs STILL HERE.
     *
     * This takes roughly two seconds total.
     */
    MPU6050_CalibrateGyro(
        &mpu1
    );


    MPU6050_CalibrateGyro(
        &mpu2
    );


    /*
     * Start Servo1 PWM.
     */
    servo1_pwm_status =
        HAL_TIM_PWM_Start(
            &htim4,
            TIM_CHANNEL_1
        );


    Servo1_SetPulse(
        SERVO1_CENTER_US
    );


    /*
     * Start Servo2 PWM.
     */
    servo2_pwm_status =
        HAL_TIM_PWM_Start(
            &htim3,
            TIM_CHANNEL_3
        );


    Servo2_SetPulse(
        SERVO2_CENTER_US
    );


    /*
     * Give everything time to settle.
     */
    HAL_Delay(
        1000
    );


    /*
     * KEEP SPOON STILL HERE.
     *
     * Current position becomes zero.
     */
    MPU6050_CalibrateReference(
        &mpu1
    );


    MPU6050_CalibrateReference(
        &mpu2
    );


    /*
     * Initialize FreeRTOS.
     */
    osKernelInitialize();


    /*
     * Create imuTask and controlTask.
     */
    MX_FREERTOS_Init();


    /*
     * Start FreeRTOS scheduler.
     */
    osKernelStart();


    /*
     * Normally we never reach this point.
     */
    while (1)
    {
    }
}


/* ============================================================
 * SYSTEM CLOCK
 *
 * HSI = 16 MHz
 * ============================================================ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef
        RCC_OscInitStruct =
        {0};


    RCC_ClkInitTypeDef
        RCC_ClkInitStruct =
        {0};


    HAL_PWREx_ControlVoltageScaling(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );


    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI;


    RCC_OscInitStruct.HSIState =
        RCC_HSI_ON;


    RCC_OscInitStruct.HSICalibrationValue =
        RCC_HSICALIBRATION_DEFAULT;


    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_NONE;


    if (
        HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;


    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_HSI;


    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;


    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;


    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV1;


    if (
        HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_0
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ============================================================
 * I2C1
 *
 * MPU1
 *
 * SCL = PA15
 * SDA = PB7
 * ============================================================ */

static void MX_I2C1_Init(void)
{
    hi2c1.Instance =
        I2C1;


    hi2c1.Init.Timing =
        0x00503D58;


    hi2c1.Init.OwnAddress1 =
        0;


    hi2c1.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;


    hi2c1.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;


    hi2c1.Init.OwnAddress2 =
        0;


    hi2c1.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;


    hi2c1.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;


    hi2c1.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;


    if (
        HAL_I2C_Init(
            &hi2c1
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_I2CEx_ConfigAnalogFilter(
            &hi2c1,
            I2C_ANALOGFILTER_ENABLE
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_I2CEx_ConfigDigitalFilter(
            &hi2c1,
            0
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ============================================================
 * I2C2
 *
 * MPU2
 *
 * SDA = PA8
 * SCL = PA9
 * ============================================================ */

static void MX_I2C2_Init(void)
{
    hi2c2.Instance =
        I2C2;


    hi2c2.Init.Timing =
        0x00503D58;


    hi2c2.Init.OwnAddress1 =
        0;


    hi2c2.Init.AddressingMode =
        I2C_ADDRESSINGMODE_7BIT;


    hi2c2.Init.DualAddressMode =
        I2C_DUALADDRESS_DISABLE;


    hi2c2.Init.OwnAddress2 =
        0;


    hi2c2.Init.OwnAddress2Masks =
        I2C_OA2_NOMASK;


    hi2c2.Init.GeneralCallMode =
        I2C_GENERALCALL_DISABLE;


    hi2c2.Init.NoStretchMode =
        I2C_NOSTRETCH_DISABLE;


    if (
        HAL_I2C_Init(
            &hi2c2
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_I2CEx_ConfigAnalogFilter(
            &hi2c2,
            I2C_ANALOGFILTER_ENABLE
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_I2CEx_ConfigDigitalFilter(
            &hi2c2,
            0
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }
}


/* ============================================================
 * TIM3
 *
 * SERVO2
 *
 * PB0 = TIM3 CH3
 *
 * 16 MHz / 16 = 1 MHz
 * 1 timer count = 1 microsecond
 *
 * period = 20000 us = 50 Hz
 * ============================================================ */

static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef
        sClockSourceConfig =
        {0};


    TIM_MasterConfigTypeDef
        sMasterConfig =
        {0};


    TIM_OC_InitTypeDef
        sConfigOC =
        {0};


    htim3.Instance =
        TIM3;


    htim3.Init.Prescaler =
        15;


    htim3.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    htim3.Init.Period =
        19999;


    htim3.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim3.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (
        HAL_TIM_Base_Init(
            &htim3
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;


    if (
        HAL_TIM_ConfigClockSource(
            &htim3,
            &sClockSourceConfig
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_TIM_PWM_Init(
            &htim3
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (
        HAL_TIMEx_MasterConfigSynchronization(
            &htim3,
            &sMasterConfig
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;


    sConfigOC.Pulse =
        SERVO2_CENTER_US;


    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;


    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;


    if (
        HAL_TIM_PWM_ConfigChannel(
            &htim3,
            &sConfigOC,
            TIM_CHANNEL_3
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    HAL_TIM_MspPostInit(
        &htim3
    );
}


/* ============================================================
 * TIM4
 *
 * SERVO1
 *
 * PB6 = TIM4 CH1
 * ============================================================ */

static void MX_TIM4_Init(void)
{
    TIM_ClockConfigTypeDef
        sClockSourceConfig =
        {0};


    TIM_MasterConfigTypeDef
        sMasterConfig =
        {0};


    TIM_OC_InitTypeDef
        sConfigOC =
        {0};


    htim4.Instance =
        TIM4;


    htim4.Init.Prescaler =
        15;


    htim4.Init.CounterMode =
        TIM_COUNTERMODE_UP;


    htim4.Init.Period =
        19999;


    htim4.Init.ClockDivision =
        TIM_CLOCKDIVISION_DIV1;


    htim4.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;


    if (
        HAL_TIM_Base_Init(
            &htim4
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    sClockSourceConfig.ClockSource =
        TIM_CLOCKSOURCE_INTERNAL;


    if (
        HAL_TIM_ConfigClockSource(
            &htim4,
            &sClockSourceConfig
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    if (
        HAL_TIM_PWM_Init(
            &htim4
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    sMasterConfig.MasterOutputTrigger =
        TIM_TRGO_RESET;


    sMasterConfig.MasterSlaveMode =
        TIM_MASTERSLAVEMODE_DISABLE;


    if (
        HAL_TIMEx_MasterConfigSynchronization(
            &htim4,
            &sMasterConfig
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    sConfigOC.OCMode =
        TIM_OCMODE_PWM1;


    sConfigOC.Pulse =
        SERVO1_CENTER_US;


    sConfigOC.OCPolarity =
        TIM_OCPOLARITY_HIGH;


    sConfigOC.OCFastMode =
        TIM_OCFAST_DISABLE;


    if (
        HAL_TIM_PWM_ConfigChannel(
            &htim4,
            &sConfigOC,
            TIM_CHANNEL_1
        )
        !=
        HAL_OK
    )
    {
        Error_Handler();
    }


    HAL_TIM_MspPostInit(
        &htim4
    );
}


/* ============================================================
 * GPIO INITIALIZATION
 * ============================================================ */

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef
        GPIO_InitStruct =
        {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /*
     * Existing PB8 output from original project.
     */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_8,
        GPIO_PIN_RESET
    );


    GPIO_InitStruct.Pin =
        GPIO_PIN_8;


    GPIO_InitStruct.Mode =
        GPIO_MODE_OUTPUT_PP;


    GPIO_InitStruct.Pull =
        GPIO_NOPULL;


    GPIO_InitStruct.Speed =
        GPIO_SPEED_FREQ_LOW;


    HAL_GPIO_Init(
        GPIOB,
        &GPIO_InitStruct
    );
}


/* ============================================================
 * ERROR HANDLER
 * ============================================================ */

void Error_Handler(void)
{
    __disable_irq();


    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
}

#endif
