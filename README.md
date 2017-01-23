# EKF_Localization

===

This is a simulation of an Extended Kalman filter for a Differential Drive Robot.
It is assumed to be driven by two independent 12V DC Motors on left and right wheels.
It is equipped with the following virtual sensors, whose signals are generated from the robot's real state (with noise):

- [x] GPS (Absolute Position)
- [x] Gyroscope (Angular Velocity)
- [x] Magnetometer (Angular Position)
- [x] Compass (Angular Position)
- [ ] Accelerometer (Linear Acceleration)
- [x] Wheel Encoder (Linear/Angular Velocity)
