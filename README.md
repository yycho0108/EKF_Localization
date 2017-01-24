# EKF_Localization

===

## Demo Video

(To Be Updated)

This is a simulation of an Extended Kalman filter for a Differential Drive Robot.
It is assumed to be driven by two independent 12V DC Motors on left and right wheels.
It is equipped with the following virtual sensors, whose signals are generated from the robot's real state (with noise):

- [x] GPS (Absolute Position)
- [x] Gyroscope (Angular Velocity)
- [x] Magnetometer (Angular Position)
- [x] Compass (Angular Position)
- [ ] Accelerometer (Linear Acceleration)
- [x] Wheel Encoder (Linear/Angular Velocity)
- [ ] Beacons (Distance from Landmark)


## Controls

Control with the mouse! The light-blue panel on the lower right shows the control panel.
The Vertical Axis controls forward/backward and the Horizontal Axis controls the angular velocity.
Press ESC, or simply close the window to quit the game.
