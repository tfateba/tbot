*****************************************************************************
** IP robot.                                                               **
*****************************************************************************

** TARGET **

The software runs on Inverted Pendulum robot.
The brain of the robot is the Arduino Mega board(mcu is atmega2560).

** The Software **

The goal is to stabilize the robot, to do that, there are several steps:
- Read data from IMU (accelerometer and gyroscope)
- Compute data to have an angle by using a kalman filter.
- Calcul the asservissement value with a PID.
- Control the robot according the offset between the mesured angle and
the target angle.

The Sofware is based on ChibiOS trunk.

** Build Procedure **
Just run make on the robot's program directory.

The demo was built using the GCC AVR toolchain.
It should build with WinAVR too!
