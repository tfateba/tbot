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
 * @note
 *          Motor Wires, connexion to the Motor Drivers
 *          Motor + is the Yellow wire.
 *          Motor - is the White wire.
 *
 */

#ifndef _IP_MOTOR_H_
#define _IP_MOTOR_H_

/*===========================================================================*/
/* Includes files.                                                           */
/*===========================================================================*/

/* Standard libraries. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
/* ChibiOS libraries. */
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
/* Local files. */
#include "ip_pid.h"
#include "ip_pwm.h"
#include "ip_conf.h"

/*===========================================================================*/
/* Enumerations, Structures and macros.                                      */
/*===========================================================================*/

/*
 * The motor driver can handle a pwm frequency up to 20kHz
 */
#define PWM_FREQUENCY 20000

#define MOTOR_L     0 /* Left motor.          */
#define MOTOR_R     1 /* Rigth motor.         */
#define MOTOR_DIR_F 2 /* Direction forward.   */
#define MOTOR_DIR_B 3 /* Direction backward.  */
#define STOP        4 /* Stop the motor.      */
#define IMU         5 /* Imu input.           */
#define JOYSTICK    6 /* Joystick input.      */

/*
 * Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no
 * prescaling so:
 * frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
 */
#define PWMVALUE F_CPU/PWM_FREQUENCY/2

/* Arduino Interruption */
#define INT2 2      /**< D19 [PD2], BLACK wire motor Right                   */
#define INT3 3      /**< D18 [PD3], RED wire Motor Left                      */

/* Left Motor encoders Ports definition. */
#define L_ENCODER_A_PORT IOPORT4 /**< Red wire.   D18 [PD3]                  */
#define L_ENCODER_B_PORT IOPORT7 /**< Black wire. D4  [PG5]                  */

/* Left Motor encoders Pins definition. */
#define L_ENCODER_A PD3 /**< Red wire.    D18 [PD3]                          */
#define L_ENCODER_B PG5 /**< Black wire.  D4  [PG5]                          */

/* Right Motor encoders Ports definition. */
#define R_ENCODER_B_PORT IOPORT5 /**< Red wire.   D5  [PE3]                  */
#define R_ENCODER_A_PORT IOPORT4 /**< Black wire. D19 [PD2]                  */

/* Right Motor encoders Pins definition. */
#define R_ENCODER_A PD2 /**< Black wire.  D19 [PD2]                          */
#define R_ENCODER_B PE3 /**< Red wire.    D5  [PE3]                          */

/* Left Motor driver Ports definition. */
#define LMD_LPWM_PORT IOPORT5  /**< Motor Forward.  D3  [PE5]                */
#define LMD_RPWM_PORT IOPORT5  /**< Motor Backward. D2  [PE4]                */
#define LMD_EN_PORT   IOPORT2  /**< Motor Enable.   D11 [PB5]                */

/* Left Motor driver Pins definition. */
#define LMD_LPWM PE5  /**< Motor Forward.   D3  [PE5]                        */
#define LMD_RPWM PE4  /**< Motor Backward.  D2  [PE4]                        */
#define LMD_EN   PB5  /**< Motor Enable.    D11 [PB5]                        */

/* Right Motor driver Ports definition. */
#define RMD_LPWM_PORT IOPORT8 /**< Motor Forward.  D6 [PH3]                  */
#define RMD_RPWM_PORT IOPORT8 /**< Motor Backward. D8 [PH5]                  */
#define RMD_EN_PORT   IOPORT8 /**< Motor Enable.   D7 [PH4]                  */

/* Right Motor driver Pins definition. */
#define RMD_LPWM PH3 /**< Motor Forward.  D6 [PH3]                           */
#define RMD_RPWM PH5 /**< Motor Backward. D8 [PH5]                           */
#define RMD_EN   PH4 /**< Motor Enable.   D7 [PH4]                           */

/*===========================================================================*/
/* Functions prototypes.                                                     */
/*===========================================================================*/

void stopMotor(uint8_t motor);
void setPWM(uint8_t motor, uint8_t direction, uint16_t dutyCycle);
void motorsStopAndReset(void);
void moveMotor(uint8_t motor, uint8_t direction, double speedRaw);
long readLeftEncoder(void);
long readRightEncoder(void);
void enableLeftMotor(void);
void enableRightMotor(void);
void disableLeftMotor(void);
void disableRightMotor(void);
void motorInit(void);
void motorGetWheelVelocity(void);

#endif /* _IP_MOTOR_H_ */
