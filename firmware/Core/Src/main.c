#include "main.h"

#include "cmsis_os2.h"

#include <math.h>

/* ============================================================

 * MPU6050 DEFINITIONS

 * ============================================================ */

#define MPU6050_ADDR              (0x68 << 1)

#define MPU6050_WHO_AM_I          0x75

#define MPU6050_PWR_MGMT_1        0x6B

#define MPU6050_CONFIG            0x1A

#define MPU6050_GYRO_CONFIG       0x1B

#define MPU6050_ACCEL_CONFIG      0x1C

#define MPU6050_ACCEL_XOUT_H      0x3B

/*

 * DLPF_CFG = 3

 * Accelerometer bandwidth is about 44 Hz.

 * Gyroscope bandwidth is about 42 Hz.

 */

#define MPU6050_DLPF_CONFIG       0x03

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

#define SERVO2_RESPONSE_TEST      0U /* Normal stabilization; identification test disabled. */

#define SERVO2_GAIN               2.0f /* Conservative unvalidated damped-controller bench setting. */
#define SERVO2_MAX_SAMPLE_AGE_MS  20U
#define SERVO2_KI_US_PER_DEG_S    1.0f
#define SERVO2_I_RATE_US_PER_S   10.0f
#define SERVO2_I_LIMIT_US        150.0f
#define SERVO2_SAFE_MIN_US        1150.0f
#define SERVO2_SAFE_MAX_US        1850.0f

/*

 * If Servo2 compensates in the wrong direction,

 * later change this between -1.0f and +1.0f.

 */

#define SERVO2_DIRECTION          (-1.0f)

#define ANGLE_DEADBAND_DEG        0.3f

/* Maximum PWM movement allowed during one 10 ms control cycle. */

#define MAX_SERVO_STEP_US         30U

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
    HAL_StatusTypeDef read_status;
    uint32_t read_count;
    uint32_t read_failures;
    uint16_t gyro_samples;
    uint16_t reference_samples;

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
volatile float servo2_command_us_f = 1500.0f;
/* Diagnostic baseline: 1 holds Servo 2 at center; 0 enables stabilization.
 * This build starts with feedback disabled; explicit bench arming is required. */
volatile uint32_t servo2_hold_center_test = 1U; /* Feedback disabled at boot after observed oscillation. */
/* Faults latch until reboot: 1 sensor invalid, 2 timing, 3 gyro clipping,
 * 4 repeated high-rate reversals. Fault holds last PWM, not a motor power cut. */
volatile uint32_t servo2_control_fault;
volatile float servo2_roll_rate_dps, servo2_d_term_us;

volatile float servo2_integral_us = 0.0f; /* Signed PWM offset, retained near zero error. */
volatile float servo2_p_term_us = 0.0f;
volatile float servo2_control_dt_s = 0.0f;
volatile float servo2_measured_roll_deg = 0.0f;
volatile float servo2_reference_roll_deg = 0.0f;
volatile float mpu2_accel_roll_deg, mpu2_gyro_x_dps, mpu2_pitch_deg;
/* Handle measurements only: do not feed MPU1 into Servo 2 control. */
volatile float mpu1_relative_roll_deg, mpu1_relative_pitch_deg;
volatile float mpu1_gyro_x_dps, mpu1_gyro_y_dps, mpu1_gyro_z_dps;
volatile uint32_t mpu1_valid, mpu1_sample_age_ms, mpu1_read_count;
volatile uint32_t mpu1_read_failures, mpu1_who_am_i;
volatile HAL_StatusTypeDef mpu1_read_status;
volatile uint32_t mpu2_read_count, mpu2_read_failures, mpu2_sample_age_ms;
volatile uint32_t mpu2_i2c_error, mpu2_who_am_i, mpu2_ok;
volatile uint32_t mpu2_gyro_samples, mpu2_reference_samples;
volatile HAL_StatusTypeDef mpu2_read_status;
volatile uint32_t servo2_inhibit = 1U, servo2_saturated, servo2_ccr3;
volatile uint32_t imu_step_ms, control_period_ms;

volatile HAL_StatusTypeDef servo1_pwm_status;

volatile HAL_StatusTypeDef servo2_pwm_status;

/* Startup-only diagnostics. Stage: 1 identity, 2 wake, 3 DLPF,
 * 4 gyro range, 5 accel range, 6 initialized. A successful retry preserves
 * last_failed_* so a transient failure is not hidden by later runtime reads.
 * HAL status: 0 OK, 1 ERROR, 2 BUSY, 3 TIMEOUT. Wrong identity can have status 0. */
typedef struct {
    uint32_t attempts;
    uint32_t stage;
    HAL_StatusTypeDef status;
    uint32_t i2c_error;
    uint32_t last_failed_stage;
    HAL_StatusTypeDef last_failed_status;
    uint32_t last_failed_i2c_error;
    uint32_t last_failed_who_am_i;
} MPU6050_InitDebug_t;
volatile MPU6050_InitDebug_t mpu1_startup, mpu2_startup;
volatile uint32_t mpu1_gyro_samples, mpu1_reference_samples;

/* Response test: 0 countdown, 1 baseline, 2 +50 us, 3 center,
 * 4 -50 us, 5 center, 6 complete, 7 aborted. Does not rearm until reset.
 * Fault: 1 invalid sensors, 2 handle moved >3 deg, 3 carriage moved >20 deg,
 * 4 control gap >50 ms, 5 explicit center-test override. */
volatile uint32_t servo2_test_state, servo2_test_fault, servo2_test_elapsed_ms;



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

static uint8_t MPU6050_Update(

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

static uint16_t Servo_LimitStep(

    uint16_t current,

    uint16_t target

);

static void Servo1_Stabilization_Update(void);

static void Servo2_Stabilization_Update(void);

/*

 * These are called by app_freertos.c.

 */

void App_IMU_TaskStep(void);

void App_Control_TaskStep(void);

/* ============================================================

 * SERVO SLEW RATE LIMITER

 * Prevents sudden PWM jumps between control cycles.

 * ============================================================ */

static uint16_t Servo_LimitStep(

    uint16_t current,

    uint16_t target

)

{

    if (target > current)

    {

        uint16_t difference = target - current;

        if (difference > MAX_SERVO_STEP_US)

        {

            return current + MAX_SERVO_STEP_US;

        }

    }

    else if (target < current)

    {

        uint16_t difference = current - target;

        if (difference > MAX_SERVO_STEP_US)

        {

            return current - MAX_SERVO_STEP_US;

        }

    }

    return target;

}

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

    uint16_t servo_target =

        (uint16_t)servo_command;

    uint16_t servo_limited =

        Servo_LimitStep(

            servo1_pulse_us,

            servo_target

        );

    Servo1_SetPulse(

        servo_limited

    );

}

/* ============================================================

 * SERVO 2 STABILIZATION

 *

 * MPU2 roll -> Servo2

 * ============================================================ */

static void Servo2_ResponseTest_Update(uint32_t now)
{
    static uint8_t started;
    static uint32_t boot_ms, phase_start_ms, previous_ms;
    static float handle_roll, handle_pitch, carriage_roll;
    uint16_t pulse = SERVO2_CENTER_US;
    uint32_t gap = now - previous_ms;
    previous_ms = now;
    if (!started) { started = 1U; boot_ms = now; }
    servo2_test_elapsed_ms = now - boot_ms;
    servo2_integral_us = 0.0f;
    servo2_p_term_us = 0.0f;
    servo2_correction = 0.0f;
    /* Give the operator 30 seconds to start logging. No feedback runs here. */
    if (servo2_test_state == 0U && servo2_test_elapsed_ms >= 30000U)
    {
        if (!mpu1_valid || servo2_inhibit) { servo2_test_fault = 1U; }
        else
        {
            handle_roll = mpu1_relative_roll_deg;
            handle_pitch = mpu1_relative_pitch_deg;
            carriage_roll = servo2_relative_angle;
            phase_start_ms = now;
            servo2_test_state = 1U;
        }
    }
    if (servo2_test_state >= 1U && servo2_test_state <= 5U)
    {
        if (!mpu1_valid || servo2_inhibit) { servo2_test_fault = 1U; }
        else if (fabsf(mpu1_relative_roll_deg - handle_roll) > 3.0f ||
                 fabsf(mpu1_relative_pitch_deg - handle_pitch) > 3.0f)
        { servo2_test_fault = 2U; }
        else if (fabsf(servo2_relative_angle - carriage_roll) > 20.0f)
        { servo2_test_fault = 3U; }
        else if (gap > 50U) { servo2_test_fault = 4U; }
        else
        {
            uint32_t t = now - phase_start_ms;
            if (t < 2000U) { servo2_test_state = 1U; }
            else if (t < 3000U) { servo2_test_state = 2U; pulse = 1550U; }
            else if (t < 5000U) { servo2_test_state = 3U; }
            else if (t < 6000U) { servo2_test_state = 4U; pulse = 1450U; }
            else if (t < 8000U) { servo2_test_state = 5U; }
            else { servo2_test_state = 6U; }
        }
    }
    if (servo2_hold_center_test && servo2_test_state < 6U)
    { servo2_test_fault = 5U; }
    if (servo2_test_fault)
    {
        servo2_test_state = 7U;
        pulse = SERVO2_CENTER_US;
    }
    servo2_command_us_f = (float)pulse;
    Servo2_SetPulse(pulse);
}

static void Servo2_Stabilization_Update(void)
{
    static uint32_t previous_ms, previous_sample, reversal_ms, reversals;
    static uint8_t previous_valid;
    static int rate_sign;
    static float filtered_rate;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - previous_ms;
    previous_ms = now;
    servo2_control_dt_s = 0.0f;
    /* Snapshot while the IMU task cannot preempt us; do not lock over I2C. */
    int32_t lock = osKernelLock();
    mpu1_relative_roll_deg = mpu1.roll_filtered_deg - mpu1.roll_reference_deg;
    mpu1_relative_pitch_deg = mpu1.pitch_filtered_deg - mpu1.pitch_reference_deg;
    mpu1_gyro_x_dps = mpu1.gyro_x_dps;
    mpu1_gyro_y_dps = mpu1.gyro_y_dps;
    mpu1_gyro_z_dps = mpu1.gyro_z_dps;
    mpu1_sample_age_ms = HAL_GetTick() - mpu1.last_update_ms;
    mpu1_read_count = mpu1.read_count;
    mpu1_read_failures = mpu1.read_failures;
    mpu1_read_status = mpu1.read_status;
    mpu1_who_am_i = mpu1.who_am_i;
    mpu1_gyro_samples = mpu1.gyro_samples;
    mpu1_reference_samples = mpu1.reference_samples;
    mpu1_valid = mpu1.ok && mpu1.filter_initialized &&
        mpu1.gyro_samples == 200U && mpu1.reference_samples == 100U &&
        mpu1.read_status == HAL_OK && mpu1.read_count != 0U &&
        mpu1_sample_age_ms <= SERVO2_MAX_SAMPLE_AGE_MS &&
        isfinite(mpu1_relative_roll_deg) && isfinite(mpu1_relative_pitch_deg) &&
        isfinite(mpu1_gyro_x_dps) && isfinite(mpu1_gyro_y_dps) &&
        isfinite(mpu1_gyro_z_dps);
    servo2_measured_roll_deg = mpu2.roll_filtered_deg;
    servo2_reference_roll_deg = mpu2.roll_reference_deg;
    mpu2_accel_roll_deg = mpu2.roll_accel_deg;
    mpu2_gyro_x_dps = mpu2.gyro_x_dps;
    mpu2_pitch_deg = mpu2.pitch_filtered_deg;
    mpu2_sample_age_ms = HAL_GetTick() - mpu2.last_update_ms;
    mpu2_read_status = mpu2.read_status;
    mpu2_read_count = mpu2.read_count;
    mpu2_read_failures = mpu2.read_failures;
    mpu2_i2c_error = hi2c2.ErrorCode;
    mpu2_who_am_i = mpu2.who_am_i;
    mpu2_ok = mpu2.ok;
    mpu2_gyro_samples = mpu2.gyro_samples;
    mpu2_reference_samples = mpu2.reference_samples;
    servo2_inhibit = !mpu2.ok || !mpu2.filter_initialized ||
        mpu2.gyro_samples != 200U || mpu2.reference_samples != 100U ||
        mpu2.read_status != HAL_OK || mpu2.read_count == 0U ||
        mpu2_sample_age_ms > SERVO2_MAX_SAMPLE_AGE_MS;
    float plane = sqrtf(mpu2.accel_x_g * mpu2.accel_x_g + mpu2.accel_z_g * mpu2.accel_z_g);
    servo2_roll_rate_dps = plane > 0.1f ?
        (mpu2.accel_z_g * mpu2.gyro_x_dps - mpu2.accel_x_g * mpu2.gyro_z_dps) / plane : 0.0f;
    if (plane <= 0.1f || !isfinite(servo2_roll_rate_dps) ||
        !isfinite(mpu2.gyro_x_dps) || !isfinite(mpu2.gyro_y_dps) || !isfinite(mpu2.gyro_z_dps)) { servo2_inhibit = 1U; }
    uint8_t gyro_clipped = fabsf(mpu2.gyro_x_dps) >= 450.0f ||
        fabsf(mpu2.gyro_y_dps) >= 450.0f || fabsf(mpu2.gyro_z_dps) >= 450.0f;
    if (lock >= 0) { osKernelRestoreLock(lock); }

    servo2_relative_angle = servo2_measured_roll_deg - servo2_reference_roll_deg;
    servo2_error = servo2_relative_angle;
    servo2_saturated = 0U;
    if (!isfinite(servo2_error)) { servo2_inhibit = 1U; }

    if (servo2_control_fault)
    {
        servo2_inhibit = 1U;
        previous_valid = 0U;
        return; /* Latched: freeze the last command; no automatic restart. */
    }
    if (servo2_hold_center_test)
    {
        previous_valid = 0U; rate_sign = 0; reversals = 0U; filtered_rate = 0.0f;
        servo2_integral_us = servo2_p_term_us = servo2_d_term_us = servo2_correction = 0.0f;
        servo2_command_us_f = (float)SERVO2_CENTER_US;
        Servo2_SetPulse(SERVO2_CENTER_US);
        return;
    }
    if (servo2_inhibit) { servo2_control_fault = 1U; return; }
    if (gyro_clipped) { servo2_control_fault = 3U; servo2_inhibit = 1U; return; }
    if (previous_valid && (elapsed_ms == 0U || elapsed_ms > 20U))
    { servo2_control_fault = 2U; servo2_inhibit = 1U; return; }
    if (previous_valid && mpu2_read_count == previous_sample) { return; }
    float dt = previous_valid ? elapsed_ms * 0.001f : 0.01f;
    if (!previous_valid) { filtered_rate = servo2_roll_rate_dps; reversal_ms = now; rate_sign = 0; reversals = 0U; }
    previous_valid = 1U; previous_sample = mpu2_read_count;
    servo2_control_dt_s = dt;
    /* Conservative bench trip: four >100 dps reversals within 0.5 seconds.
     * May also trip on vigorous hand motion; it is not a tremor classifier. */
    if (now - reversal_ms > 500U) { reversals = 0U; rate_sign = 0; reversal_ms = now; }
    int sign = servo2_roll_rate_dps > 100.0f ? 1 : servo2_roll_rate_dps < -100.0f ? -1 : 0;
    if (sign && sign != rate_sign)
    {
        if (rate_sign) { ++reversals; }
        rate_sign = sign;
        if (reversals >= 4U) { servo2_control_fault = 4U; servo2_inhibit = 1U; return; }
    }
    filtered_rate += dt / (0.03f + dt) * (servo2_roll_rate_dps - filtered_rate);
    if (fabsf(servo2_error) < ANGLE_DEADBAND_DEG) { servo2_error = 0.0f; }
    servo2_correction = SERVO2_GAIN * servo2_error;
    servo2_p_term_us = SERVO2_DIRECTION * servo2_correction;
    servo2_d_term_us = SERVO2_DIRECTION * 0.08f * filtered_rate;
    if (servo2_d_term_us > 20.0f) { servo2_d_term_us = 20.0f; }
    if (servo2_d_term_us < -20.0f) { servo2_d_term_us = -20.0f; }
    float rate = SERVO2_DIRECTION * SERVO2_KI_US_PER_DEG_S * servo2_error;
    if (rate > SERVO2_I_RATE_US_PER_S) { rate = SERVO2_I_RATE_US_PER_S; }
    if (rate < -SERVO2_I_RATE_US_PER_S) { rate = -SERVO2_I_RATE_US_PER_S; }
    float delta = rate * dt;
    float candidate = servo2_integral_us + delta;
    if (candidate > SERVO2_I_LIMIT_US) { candidate = SERVO2_I_LIMIT_US; }
    if (candidate < -SERVO2_I_LIMIT_US) { candidate = -SERVO2_I_LIMIT_US; }
    float proposed = SERVO2_CENTER_US + servo2_p_term_us + servo2_d_term_us + candidate;
    float step = 600.0f * dt; /* 6 us per nominal control tick. */
    float previous = (float)servo2_pulse_us;
    if (!((delta > 0.0f && (proposed > SERVO2_SAFE_MAX_US || proposed > previous + step)) ||
          (delta < 0.0f && (proposed < SERVO2_SAFE_MIN_US || proposed < previous - step))))
    { servo2_integral_us = candidate; }
    proposed = SERVO2_CENTER_US + servo2_p_term_us + servo2_d_term_us + servo2_integral_us;
    if (proposed < SERVO2_SAFE_MIN_US) { proposed = SERVO2_SAFE_MIN_US; servo2_saturated = 1U; }
    if (proposed > SERVO2_SAFE_MAX_US) { proposed = SERVO2_SAFE_MAX_US; servo2_saturated = 1U; }
    if (proposed > previous + step) { proposed = previous + step; }
    if (proposed < previous - step) { proposed = previous - step; }
    servo2_command_us_f = proposed;
    Servo2_SetPulse((uint16_t)(proposed + 0.5f));
}


/* ============================================================

 * MPU6050 INITIALIZATION

 * ============================================================ */

static void MPU6050_TryInit(

    MPU6050_t *mpu, volatile MPU6050_InitDebug_t *debug

)

{

    uint8_t wake = 0x00;

    uint8_t gyro_config = 0x08;   /* +/-500 deg/s */

    uint8_t accel_config = 0x00;  /* +/-2g */

    uint8_t dlpf_config = MPU6050_DLPF_CONFIG;

    HAL_StatusTypeDef status;

    mpu->ok = 0U;
    mpu->who_am_i = 0U;

    /* Check MPU identity. */

    debug->stage = 1U;
    status = HAL_I2C_Mem_Read(

        mpu->i2c,

        MPU6050_ADDR,

        MPU6050_WHO_AM_I,

        I2C_MEMADD_SIZE_8BIT,

        &mpu->who_am_i,

        1,

        100

    );
    debug->status = status;
    debug->i2c_error = mpu->i2c->ErrorCode;

    if (status == HAL_OK && mpu->who_am_i == 0x68)

    {

        /* Identity accepted; ok is set only after all configuration succeeds. */

    }

    else

    {

        mpu->ok = 0;

        return;

    }

    /* Wake MPU6050. */

    debug->stage = 2U;
    status = HAL_I2C_Mem_Write(

        mpu->i2c,

        MPU6050_ADDR,

        MPU6050_PWR_MGMT_1,

        I2C_MEMADD_SIZE_8BIT,

        &wake,

        1,

        100

    );
    debug->status = status;
    debug->i2c_error = mpu->i2c->ErrorCode;

    if (status != HAL_OK)

    {

        mpu->ok = 0;

        return;

    }

    HAL_Delay(100);

    /* Configure the MPU6050 hardware digital low pass filter. */

    debug->stage = 3U;
    status = HAL_I2C_Mem_Write(

        mpu->i2c,

        MPU6050_ADDR,

        MPU6050_CONFIG,

        I2C_MEMADD_SIZE_8BIT,

        &dlpf_config,

        1,

        100

    );
    debug->status = status;
    debug->i2c_error = mpu->i2c->ErrorCode;

    if (status != HAL_OK)

    {

        mpu->ok = 0;

        return;

    }

    /* Configure gyroscope. */

    debug->stage = 4U;
    status = HAL_I2C_Mem_Write(

        mpu->i2c,

        MPU6050_ADDR,

        MPU6050_GYRO_CONFIG,

        I2C_MEMADD_SIZE_8BIT,

        &gyro_config,

        1,

        100

    );
    debug->status = status;
    debug->i2c_error = mpu->i2c->ErrorCode;

    if (status != HAL_OK)

    {

        mpu->ok = 0;

        return;

    }

    /* Configure accelerometer. */

    debug->stage = 5U;
    status = HAL_I2C_Mem_Write(

        mpu->i2c,

        MPU6050_ADDR,

        MPU6050_ACCEL_CONFIG,

        I2C_MEMADD_SIZE_8BIT,

        &accel_config,

        1,

        100

    );
    debug->status = status;
    debug->i2c_error = mpu->i2c->ErrorCode;

    if (status != HAL_OK)

    {

        mpu->ok = 0;

        return;

    }

    HAL_Delay(100);
    mpu->ok = 1U;
    debug->stage = 6U;
}

/* Startup only: release a slave stranded mid-byte without driving a line high.
 * result: 1 clear, 2 SCL held low, 3 SDA held low, 4 peripheral restore failed. */
typedef struct { uint32_t attempts, pulses, result; } MPU6050_BusDebug_t;
volatile MPU6050_BusDebug_t mpu1_bus_recovery, mpu2_bus_recovery;

static uint8_t MPU6050_ClearBus(MPU6050_t *mpu)
{
    volatile MPU6050_BusDebug_t *d =
        (mpu == &mpu1) ? &mpu1_bus_recovery : &mpu2_bus_recovery;
    GPIO_TypeDef *scl_port = GPIOA;
    GPIO_TypeDef *sda_port = (mpu == &mpu1) ? GPIOB : GPIOA;
    uint16_t scl = (mpu == &mpu1) ? GPIO_PIN_15 : GPIO_PIN_9;
    uint16_t sda = (mpu == &mpu1) ? GPIO_PIN_7 : GPIO_PIN_8;
    GPIO_InitTypeDef pins = {0};
    ++d->attempts; d->pulses = 0U; d->result = 0U;
    if (HAL_I2C_DeInit(mpu->i2c) != HAL_OK) { d->result = 4U; return 0U; }
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* Preload released levels before switching to open-drain GPIO. */
    HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda_port, sda, GPIO_PIN_SET);
    pins.Mode = GPIO_MODE_OUTPUT_OD; pins.Pull = GPIO_NOPULL;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    pins.Pin = scl; HAL_GPIO_Init(scl_port, &pins);
    pins.Pin = sda; HAL_GPIO_Init(sda_port, &pins);
    HAL_Delay(1);
    if (HAL_GPIO_ReadPin(scl_port, scl) == GPIO_PIN_RESET) { d->result = 2U; }
    while (!d->result && HAL_GPIO_ReadPin(sda_port, sda) == GPIO_PIN_RESET && d->pulses < 9U)
    {
        HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_SET); HAL_Delay(1);
        ++d->pulses;
        if (HAL_GPIO_ReadPin(scl_port, scl) == GPIO_PIN_RESET) { d->result = 2U; }
    }
    if (!d->result)
    {
        /* STOP: release SDA while SCL is high. */
        HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(sda_port, sda, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_SET); HAL_Delay(1);
        if (HAL_GPIO_ReadPin(scl_port, scl) == GPIO_PIN_RESET) { d->result = 2U; }
        HAL_GPIO_WritePin(sda_port, sda, GPIO_PIN_SET); HAL_Delay(1);
        if (!d->result) { d->result = HAL_GPIO_ReadPin(sda_port, sda) == GPIO_PIN_SET ? 1U : 3U; }
    }
    /* Always release both pins and restore the original alternate-function bus. */
    HAL_GPIO_WritePin(scl_port, scl, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda_port, sda, GPIO_PIN_SET);
    if (HAL_I2C_Init(mpu->i2c) != HAL_OK ||
        HAL_I2CEx_ConfigAnalogFilter(mpu->i2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK ||
        HAL_I2CEx_ConfigDigitalFilter(mpu->i2c, 0U) != HAL_OK) { d->result = 4U; }
    return d->result == 1U;
}

static void MPU6050_Init(MPU6050_t *mpu)
{
    volatile MPU6050_InitDebug_t *debug =
        (mpu == &mpu1) ? &mpu1_startup : &mpu2_startup;
    *debug = (MPU6050_InitDebug_t){0};
    /* Allow sensor power to settle, then retry only before the scheduler starts.
     * Do not recalibrate or introduce blocking recovery in the running loop. */
    HAL_Delay(100);
    for (uint32_t attempt = 1U; attempt <= 3U; ++attempt)
    {
        debug->attempts = attempt;
        MPU6050_TryInit(mpu, debug);
        if (mpu->ok) { return; }
        debug->last_failed_stage = debug->stage;
        debug->last_failed_status = debug->status;
        debug->last_failed_i2c_error = debug->i2c_error;
        debug->last_failed_who_am_i = mpu->who_am_i;
        if (attempt < 3U)
        {
            if ((debug->i2c_error & (HAL_I2C_ERROR_TIMEOUT | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_BERR)) ||
                debug->status == HAL_BUSY)
            {
                if (!MPU6050_ClearBus(mpu)) { return; }
            }
            HAL_Delay(50);
        }
    }
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

    mpu->gyro_samples = successful_samples;
}

/* ============================================================

 * READ MPU + COMPLEMENTARY FILTER

 * ============================================================ */

static uint8_t MPU6050_Update(

    MPU6050_t *mpu

)

{

    if (!mpu->ok)

    {

        return 0U;

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

            5 /* Bound the polling timeout during the 5 ms IMU period. */

        );

    mpu->read_status = status;
    if (status != HAL_OK)

    {

        mpu->read_failures++;
        return 0U;

    }

    mpu->read_count++;

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

            /* After a gap, re-seed instead of integrating a fictional 5 ms. */
            mpu->filter_initialized = 0U;
            dt = 0.005f;

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

    /* These are bounded accelerometer tilt angles, not full Euler angles.
     * Differentiate their definitions using dg/dt = -omega cross g.
     * Using raw gx/gy directly reverses the fast estimate when az is negative.
     * Keep the existing static angle convention and servo feedback direction. */
    const float roll_plane = sqrtf(mpu->accel_x_g * mpu->accel_x_g +
                                   mpu->accel_z_g * mpu->accel_z_g);
    const float pitch_plane = sqrtf(mpu->accel_y_g * mpu->accel_y_g +
                                    mpu->accel_z_g * mpu->accel_z_g);
    const float roll_tilt_rate = roll_plane > 0.1f ?
        (mpu->accel_z_g * mpu->gyro_x_dps - mpu->accel_x_g * mpu->gyro_z_dps) / roll_plane : 0.0f;
    const float pitch_tilt_rate = pitch_plane > 0.1f ?
        (mpu->accel_z_g * mpu->gyro_y_dps - mpu->accel_y_g * mpu->gyro_z_dps) / pitch_plane : 0.0f;

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

                    pitch_tilt_rate *

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

                    roll_tilt_rate *

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

    return 1U;
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

        if (MPU6050_Update(mpu))

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

    mpu->reference_samples = samples;
}

/* ============================================================

 * FREERTOS IMU TASK STEP

 *

 * app_freertos.c calls this at 200 Hz.

 * ============================================================ */

void App_IMU_TaskStep(void)
{
    uint32_t start = HAL_GetTick();
    /* Read handle first so the carriage sample is newest for control.
     * Existing timeout/overrun diagnostics must be checked with both buses active. */
    MPU6050_Update(&mpu1);
    MPU6050_Update(&mpu2);
    imu_step_ms = HAL_GetTick() - start;
    imu_task_count++;
}

/* ============================================================

 * FREERTOS CONTROL TASK STEP

 *

 * app_freertos.c calls this at 100 Hz.

 * ============================================================ */

void App_Control_TaskStep(void)
{
    static uint32_t last_control_ms;
    uint32_t now = HAL_GetTick();
    control_period_ms = now - last_control_ms;
    last_control_ms = now;
    /*
     * Servo 1 is held at center while Servo 2 is tuned.
     */
    Servo1_SetPulse(SERVO1_CENTER_US);

    /*
     * Real Servo 2 stabilization is active.
     */
    if (servo2_pwm_status == HAL_OK)
    {
        Servo2_Stabilization_Update();
    }

    servo2_ccr3 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
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

        3002; /* 1 MHz / 3003 = 333 Hz; DS215MG V8.0 only. */

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{

    if (htim->Instance == TIM6)

    {

        HAL_IncTick();

    }

}

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
