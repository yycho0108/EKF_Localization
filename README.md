# EKF\_Localization

===

## Demo Video

[Youtube Link](https://youtu.be/q2WupFdK0Eo)

(To Be Updated)

This is a simulation of an Extended Kalman filter for a Differential Drive Robot.
It is assumed to be driven by two independent 12V DC Motors on left and right wheels.
It is equipped with the following virtual sensors, whose signals are generated from the robot's real state (with noise):

- [x] GPS (Absolute Position)
- [x] Gyroscope (Angular Velocity)
- [x] Magnetometer (Angular Position)
- [x] Compass (Angular Position)
- [ ] Accelerometer (Linear Acceleration) - Requires More States to Track
- [x] Wheel Encoder (Linear/Angular Velocity)
- [x] Beacons (Distance from Landmark)


## Controls

Control with the mouse! The light-blue panel on the lower right shows the control panel.
The Vertical Axis controls forward/backward and the Horizontal Axis controls the angular velocity.
Press ESC, or simply close the window to quit the game.
