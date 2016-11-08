/**
 *
 * @file    ip_motor.h
 *
 * @brief   motor driver header.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    07 Septembre 2015
 *
 * @update  28 September 2016
 *
 * @note
 *          Motor Wires, connexion to the Motor Drivers
 *          Motor + is the Yellow wire.
 *          Motor - is the White wire.
 *
 * @version 1.2
 *
 */

#ifndef _IP_MOTOR_H_
#define _IP_MOTOR_H_

/*=========================================================================*/
/* Includes files.                                                         */
/*=========================================================================*/
#include <math.h>
#include "hal.h"
#include "ip_pid.h"

/*=========================================================================*/
/* Enumerations, Structures and macros.                                    */
/*=========================================================================*/

/*
 * The motor driver can handle a pwm frequency up to 20kHz
 */
#define PWM_FREQUENCY 20000

#define MOTOR_L     0 // Left motor
#define MOTOR_R     1 // Rigth motor
#define MOTOR_DIR_F 2 // Direction forward
#define MOTOR_DIR_B 3 // Direction backward
#define STOP        4 // Stop the motor
#define IMU         5 // Imu input
#define JOYSTICK    6 // Joystick input

/*
 * Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no
 * prescaling so:
 * frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
 */
#define PWMVALUE F_CPU/PWM_FREQUENCY/2

/* Arduino Interruption */
#define INT2 2      /**< On Pin 19 [PD2] .                                 */
#define INT3 3      /**< On Pin 18 [PD3].                                  */

/* Motor encoders Ports on arduino Mega.*/
#define L_ENCODER_A_PORT IOPORT4 /**< Red wire. PE4                        */
#define R_ENCODER_B_PORT IOPORT4 /**< Red wire. PE5                        */
#define L_ENCODER_B_PORT IOPORT7 /**< Black wire. PG5                      */
#define R_ENCODER_A_PORT IOPORT5 /**< Black wire. PE3                      */

/* Motor encoders Pins on Arduino Mega. */
#define L_ENCODER_A PD2 /**< Red wire. D19                                 */
#define R_ENCODER_B PD3 /**< Red wire. D18                                 */
#define L_ENCODER_B PG5 /**< Black wire. PG5                               */
#define R_ENCODER_A PE3 /**< Black wire. PE3                               */

/* Left Motor driver Ports. */
#define LMD_LPWM_PORT IOPORT8  /**< Motor Forward.  PH3                    */
#define LMD_RPWM_PORT IOPORT8  /**< Motor Backward. PH5                    */
#define LMD_EN_PORT   IOPORT8  /**< Motor Enable.   PH4                    */

/* Right Motor driver Pins*/
#define LMD_LPWM PH3  /**< Motor Forward.   PH3                            */
#define LMD_RPWM PH5  /**< Motor Backward.  PH5                            */
#define LMD_EN   PH4  /**< Motor Enable.    PH4                            */

/* Left Motor driver */
#define RMD_LPWM_PORT IOPORT5 /**< Motor Forward.  PE4                     */
#define RMD_RPWM_PORT IOPORT5 /**< Motor Backward. PE5                     */
#define RMD_EN_PORT   IOPORT2 /**< Motor Enable.   PB4                     */

/* Left Motor driver */
#define RMD_LPWM PE4 /**< Motor Forward.  PE4                              */
#define RMD_RPWM PE5 /**< Motor Backward. PE5                              */
#define RMD_EN   PB4 /**< Motor Enable.   PB4                              */

/*=========================================================================*/
/* Functions prototypes.                                                   */
/*=========================================================================*/

/**
 * @fn    stopMotor
 * @brief Stop the corresponding motor.
 *
 * @param[in] motor The motor to stop, rigth or left.
 */
void stopMotor(uint8_t motor);

/**
 * @fn    setPWM
 * @brief Generate the corresponding PWM for speed control.
 *
 * @param[in] motor     The motor to pilot, rigth or left.
 * @param[in] direction The direction of the motor, backward or forward.
 * @param[in] dutyCycle The duty cycle to set the pwm.
 */
void setPWM(uint8_t motor, uint8_t direction, int dutyCycle);

/**
 * @fn    motorsStopAndReset
 * @brief Stop both motors and initialize wheels position, PID parameters.
 */
void motorsStopAndReset(void);

/**
 * @fn    moveMotor
 * @brief Driving the motor to set to the given speed.
 *
 * @param[in] motor     The motor to pilot, rigth or left.
 * @param[in] direction The direction of the motor, backward or forward.
 * @param[in] speedRaw  The speed to set the motor.
 */
void moveMotor(uint8_t motor, uint8_t direction, double speedRaw);

/**
 * @fn    cbLeftEncoder
 * @brief The left encoder callback function.
 */
//void cbLeftEncoder(void);

/**
 * @fn    cbRightEncoder
 * @brief The right encoder callback function.
 */
//void cbRightEncoder(void);

/**
 * @fn    readLeftEncoder
 * @brief The encoders decrease when motor is traveling forward and increase
 *        when traveling backward.
 *
 * @return  leftCounter The value of the left encoder.
 */
long readLeftEncoder(void);

/**
 * @fn    readRightEncoder
 * @brief The encoders decrease when motor is traveling forward and increase
 *        when traveling backward.
 *
 * @return  rightCounter The value of the right encoder.
 */
long readRightEncoder(void);

/**
 * @fn    motorInit
 * @brief Initialize all pins needs for motor control
 */
void motorInit(void);

/**
 * @fn    motorGetWheelVelocity
 * @brief Get the wheel velocity for asservissement routine.
 */
void motorGetWheelVelocity(void);


// TODO: Remove this test functions define just for test purpose.

/**
 * @fn      readLeftEncoderStateA
 * @brief   return the state of the rigth encder A.
 *
 * @return  rightCounter the value of the right encoder
 */
long readLeftEncoderStateA(void);

/**
 * @fn      readLeftEncoderStateB
 * @brief   return the state of the rigth encder A.
 *
 * @return  rightCounter the value of the right encoder
 */
long readLeftEncoderStateB(void);

/**
 * @fn      readRightEncoderStateA
 * @brief   return the state of the rigth encder A.
 *
 * @return  rightCounter the value of the right encoder
 */
long readRightEncoderStateA(void);

/**
 * @fn      readRightEncoderStateB
 * @brief   return the state of the rigth encder A.
 *
 * @return  rightCounter the value of the right encoder
 */
long readRightEncoderStateB(void);

#endif
