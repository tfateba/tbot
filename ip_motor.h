/**
 *
 * @file    ip_motor.h
 *
 * @brief   motor driver header file.
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

#ifndef IP_MOTOR_H
#define IP_MOTOR_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/* Standard libraries. */
#include <stdint.h>

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/*
 * The motor driver can handle a pwm frequency up to 20kHz
 */
#define PWM_FREQUENCY 20000

#define MOTOR_L     0 /* Left motor         */
#define MOTOR_R     1 /* Rigth motor        */
#define MOTOR_DIR_F 2 /* Direction forward  */
#define MOTOR_DIR_B 3 /* Direction backward */
#define STOP        4 /* Stop the motor     */
#define IMU         5 /* Imu input          */
#define JOYSTICK    6 /* Joystick input.    */

/*
 * Frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, we use no
 * prescaling so:
 * frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2
 */
#define PWMVALUE          F_CPU/PWM_FREQUENCY/2

#define INT2              2      /**< External interruption 2     D19 [PD2] */
#define INT3              3      /**< External interruption 3     D18 [PD3] */

#define L_ENCODER_A_PORT  IOPORT4 /**< Left Motor coder A Port.   D18 [PD3] */
#define L_ENCODER_B_PORT  IOPORT7 /**< Left Motor coder B Port.   D4  [PG5] */
#define R_ENCODER_B_PORT  IOPORT5 /**< Right Motor coder A Port.  D5  [PE3] */
#define R_ENCODER_A_PORT  IOPORT4 /**< Right Motor coder B Port.  D19 [PD2] */
#define LMD_LPWM_PORT     IOPORT5 /**< Left Motor Forward port.   D3  [PE5] */
#define LMD_RPWM_PORT     IOPORT5 /**< Left Motor Backward port.  D2  [PE4] */
#define LMD_EN_PORT       IOPORT2 /**< Left Motor Enable port.    D11 [PB5] */
#define RMD_LPWM_PORT     IOPORT8 /**< Right Motor Forward port.  D6 [PH3]  */
#define RMD_RPWM_PORT     IOPORT8 /**< Right Motor Backward port. D8 [PH5]  */
#define RMD_EN_PORT       IOPORT8 /**< Right Motor Enable port.   D7 [PH4]  */
#define L_ENCODER_A       PD3     /**< Left Motor coder A Pin.    D18 [PD3] */
#define L_ENCODER_B       PG5     /**< Left Motor coder B Pin.    D4  [PG5] */
#define R_ENCODER_A       PD2     /**< Right Motor coder A Pin.   D19 [PD2] */
#define R_ENCODER_B       PE3     /**< Right Motor coder B Pin.   D5  [PE3] */
#define LMD_LPWM          PE5     /**< Left Motor Forward pin.    D3  [PE5] */
#define LMD_RPWM          PE4     /**< Left Motor Backward pin.   D2  [PE4] */
#define LMD_EN            PB5     /**< Left Motor Enable pin.     D11 [PB5] */
#define RMD_LPWM          PH3     /**< Right Motor Forward pin.   D6 [PH3]  */
#define RMD_RPWM          PH5     /**< Right Motor Backward pin.  D8 [PH5]  */
#define RMD_EN            PH4     /**< Right Motor Enable pin.    D7 [PH4]  */ 

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void motorsStop(uint8_t motor);
void motorsSetPWM(uint8_t motor, uint8_t direction, uint16_t dutyCycle);
void motorsStopAndReset(void);
void motorsMove(uint8_t motor, uint8_t direction, double speedRaw);
uint32_t motorsReadLeftEncoder(void);
uint32_t motorsReadRightEncoder(void);
void motorsLeftEnable(void);
void motorsRightEnable(void);
void motorsLeftDisable(void);
void motorsRightDisable(void);
void motorsInit(void);
void motorsGetWheelVelocity(void);

#endif /* IP_MOTOR_H  */

