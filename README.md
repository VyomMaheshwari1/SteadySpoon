SteadySpoon



SteadySpoon is a prototype active-stabilization utensil that I'm building to experiment with reducing tremor-like hand motion.



The current setup uses an STM32G431 and an MPU6050. So far I've implemented I2C sensor communication, gyro calibration, tilt estimation, accelerometer/gyro fusion using a complementary filter, and PWM setup for the servos.



Right now I'm working on getting the first physical servo axis running.



\## Hardware



\- STM32 NUCLEO-G431KB

\- MPU6050 IMUs

\- KST DS215MG servos

\- External servo power supply



\## Current progress



\- STM32 + MPU6050 communication working

\- Real accelerometer and gyro readings working

\- Gyro bias calibration implemented

\- Complementary-filter angle estimation working

\- Servo PWM configured

\- Physical servo testing next



This is still a work in progress. CAD, test data, and stabilization results will be added as the prototype develops.

