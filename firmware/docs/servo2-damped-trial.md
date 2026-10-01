# Servo 2 damped controller trial

This revision is an unvalidated bench controller. It starts with Servo 2 held at 1500 us and Servo 1 centered. Physical stabilization performance has not been demonstrated. Do not mistake hold-center mode for active stabilization.

## Changes

- Correct gyro-rate projection to match the bounded accelerometer tilt definitions, including inverted sensor mounting.
- Retry MPU startup with bounded, open-drain I2C bus recovery and retained diagnostics.
- Use angle gain 2 us/degree, rate damping 0.08 us/(degree/s), and a 30 ms rate filter. Damping is limited to +/-20 us.
- Use integral gain 1 us/(degree*s), limited to 10 us/s and +/-150 us, with anti-windup.
- Limit commands to 1150–1850 us and nominal changes to 6 us per 10 ms control tick.
- Configure Servo 2 TIM3 for approximately 333 Hz PWM with 1 us resolution, for the KST DS215MG V8.0. Servo 1 timing is unchanged.

## Defaults and fault behavior

`servo2_hold_center_test` defaults to 1. Enabling feedback requires an explicit, supervised bench action. Automatic actuator testing is disabled.

`servo2_control_fault` latches until reset:

| Code | Trigger |
|---|---|
| 1 | Invalid, stale, nonfinite, or singular-coordinate sensor data |
| 2 | Active control interval outside the permitted timing window |
| 3 | Any carriage gyro axis reaches 450 degrees/second |
| 4 | Four rate reversals exceeding +/-100 degrees/second within 500 ms |

A fault freezes the last PWM command. It does **not** disconnect motor power. The oscillation guard can also trip on vigorous imposed motion; it is not a tremor classifier. Reset returns to hold-center mode.

Keep external servo power off when flashing. Refresh CubeMonitor variable addresses from the exact newly flashed ELF; rebuilding can move globals. Never use a stale zero-valued graph as evidence of stabilization. Include fault, inhibit, hold-center and sensor-health fields in subsequent recordings.

## Validation

ARM firmware build passed. Eleven isolated host tests exercise the actual controller functions: startup hold, command slew/integral limits, damping direction and decay, clipping/reversal latches, timing, stale data, anti-windup, timer wrap, duplicate samples and nonfinite input. Run `python firmware/tests/test_servo2_controller.py` with a host GCC compiler installed (`CC` can select it).

Replay of a prior unstable recording triggered the reversal guard and held the output. Replay verifies response to recorded inputs, not new closed-loop physical behavior. Hardware validation is still required. Known build warnings concern unused Servo 1 stabilization and the legacy automatic-response helper.

Reference: [KST DS215MG V8.0 specification](https://www.kst-servo-shop.de/media/9b/d5/38/1751020226/KST_0821_DS215MG_V8_Datenblatt_04_2025_de.pdf).
