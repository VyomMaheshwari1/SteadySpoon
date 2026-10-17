# Servo 2 handle feedforward bench trial

The small open-loop +/-20 us diagnostic completed without a fault and the user reported smooth motion. Positive 20 us produced approximately +1.6 degrees carriage roll; negative 20 us about -1.5 degrees. This is a local static calibration, not validated bandwidth or a global linkage map.

Current candidate uses no PID: command = 1500 + 13 * (handle roll - handle roll at arming), in microseconds. Observed handle/carriage signs are opposite; positive PWM increases carriage roll. MPU2 monitors actual carriage movement but does not change the output. This preserves the starting orientation, not automatically gravity-level. Place the spoon level before arming if level is desired.

Boot is unarmed and commands center. Explicit servo2_test_request=1 captures starting handle/carriage orientation. Handle roll command is clipped to +/-5 degrees (+/-65 us); output ramps at 100 us/s, the same ramp tested in the diagnostic. At 30 seconds it smoothly returns to center (up to 0.65 seconds), then cannot rearm until reset. Servo 1 remains centered. This deliberately slow candidate is for validating tilt compensation, not claiming tremor suppression.

State 0 idle, 1 active, 2 returning to center, 6 complete, 7 fault. Fault 1 sensor invalid, 2 handle roll exceeds +/-8 degrees or pitch exceeds +/-10 degrees from arming, 3 carriage exceeds +/-8 degrees or observed carriage X/projected roll rate exceeds 100 degrees/s, 4 scheduling gap exceeds 20 ms, 5 request cleared while active, 6 invalid request. Any fault freezes the last PWM, not a motor power cut. Cut external servo power on a jerk or fault. Reset required to repeat. The old hold-center flag does not arm or disarm this build; use the request and state fields.

Firmware source and ELF for the successful prior diagnostic are archived in the task workspace outputs/handle-feedforward/diagnostic-before.*. Refresh monitor symbols from the newly flashed ELF. Verify physical improvement by comparing carriage angle change against handle angle change while holding a small tilt; no source-code test proves the linkage calibration, axis mapping or actual stabilization.

Host tests compile actual firmware functions and exercise no automatic arming, both command directions, limits and nominal slew, timed return, no repeat arming, clock wrap, sensor faults, motion/rate bounds, timing faults, cancellation and malformed requests.

The temporary cross-axis pitch allowance was widened from 2 to 10 degrees after hand-held tests repeatedly aborted on small coupled movements. This does not enable a second controlled axis or validate compensation at large pitch angles. Host cases check acceptance at +/-10 degrees and a latched abort beyond the allowance.

## Measured trial on October 16, 2026

The 30-second trial completed at state 6, fault 0, request 0 and 1500 us. In the moving interval (2 to 12 seconds after first observed active sample), handle angular standard deviation was 1.756 degrees and carriage standard deviation was 1.168 degrees, a 33.5 percent reduction relative to handle variation. Peak-to-peak ranges were 7.875 and 5.633 degrees respectively. This is an input/output comparison, not attenuation against a matched disabled-controller trial. During the quiet interval from 14 to 29 seconds, carriage standard deviation was 0.0179 degrees.

The host recording median interval was approximately 0.193 seconds (5.17 Hz). It cannot validate tremor-frequency suppression, peak gyro speed, or accurate control latency. Commands frequently approached the configured 100 us/s limit. A future speed-limit change must be compared using the same small input amplitude and rate before increasing imposed movement frequency. Both-axis stabilization and fast-shaking performance remain unvalidated.
