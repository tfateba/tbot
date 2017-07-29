******************************************************************************
** IP robot.                                                                **
******************************************************************************

*****************
** Description **
*****************

IP robot is a self balancing robot.
It goal is to stay up, for that, it use an hardware based on many electronic
modules (MCU board, IMU, MOTOR ...)

On top of the hardware, there is the software part base on a famous
real time operating system build for small microcontroller.

ChibiOS/RT is use to archive all operations needed to stabilize the robot.

To see the robot in action the link is bellow:
https://www.youtube.com/watch?v=Zo3fDoTIZaA

************
** TARGET **
************

The software runs on Inverted Pendulum robot.
The brain of the robot is the Arduino Mega board(mcu is atmega2560).

******************
** The Software **
******************

To stabilize the robot, the following steps are used:
- Read data from IMU (accelerometer and gyroscope)
- Compute data to have an angle by using a kalman filter.
- Calcul the asservissement value with a PID.
- Control the robot according the offset between measured and targeted angle.

The Sofware is based on ChibiOS trunk.

*********************
** Build Procedure **
*********************
Just run make on the robot's program directory.

The demo was built using the GCC AVR toolchain.
It should build with WinAVR too!

