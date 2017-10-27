
/**
 *
 * @file    ipmotor.h
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

#ifndef IPMOTOR_H
#define IPMOTOR_H

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/**
 * @brief Motors enumerations
 */
typedef enum {
  MOTOR_L,      /**< Left motor.                */
  MOTOR_R,      /**< Right motor.               */
  MOTOR_DIR_F,  /**< Motor forward direction.   */
  MOTOR_DIR_B,  /**< Motor backward direction.  */
}motor_e;

/**
 * @brief Encoders enumerations
 */
typedef enum {
  ENCODER_L,          /**< Left encoder.                  */
  ENCODER_R,          /**< Right encoder.                 */
  ENCODER_L_STATE_A,  /**< State A of the left encoder.   */
  ENCODER_L_STATE_B,  /**< State B of the left encoder.   */
  ENCODER_R_STATE_A,  /**< State A of the right encoder.  */
  ENCODER_R_STATE_B   /**< State B of the right encoder.  */
}encoder_e;

#define LMD_LPWM_PORT     IOPORT5 /**< Left motor driver forward pwm port.  */
#define LMD_RPWM_PORT     IOPORT5 /**< Left motor driver backwad pwm port.  */
#define LMD_EN_PORT       IOPORT2 /**< Left motor enable port.              */

#define LMD_LPWM          PE5     /**< Left motor driver forward pwm pin.   */
#define LMD_RPWM          PE4     /**< Left motor driver backwad pwm pin.   */
#define LMD_EN            PB5     /**< Left motor enable pin.               */

#define RMD_LPWM_PORT     IOPORT8 /**< Right motor driver forward port.     */
#define RMD_RPWM_PORT     IOPORT8 /**< Right motor driver backward port.    */
#define RMD_EN_PORT       IOPORT8 /**< Right motor driver enable port.      */

#define RMD_LPWM          PH3     /**< Right motor driver forward pin.      */
#define RMD_RPWM          PH5     /**< Right motor driver backward pin.     */
#define RMD_EN            PH4     /**< Right motor driver enable pin.       */

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void motorStop(uint8_t motor);
void motorsStopAndReset(void);
void motorMove(uint8_t motor, uint8_t direction, float speedRaw);
void motorInit(void);
void motorEnable(motor_e motor);
void motorDisable(motor_e motor);

#endif /* IPMOTOR_H */

