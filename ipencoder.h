
/**
 *
 * @file    ipencoder.h
 *
 * @brief   Encoder header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    27 August 2017
 *
 */

#ifndef IPENCODER_H
#define IPENCODER_H

/* Arduino Interruption */
#define INT2 2      /**< D19 [PD2], BLACK wire motor Right                  */
#define INT3 3      /**< D18 [PD3], RED wire Motor Left                     */

#define L_ENCODER_A_PORT  IOPORT4 /**< Left encoder A port.                 */
#define L_ENCODER_B_PORT  IOPORT7 /**< Left encoder B port.                 */

#define L_ENCODER_A       PD3     /**< Left encoder A pin.                  */
#define L_ENCODER_B       PG5     /**< Left encoder B pin.                  */

#define R_ENCODER_B_PORT  IOPORT5 /**< Right encoder B port.                */
#define R_ENCODER_A_PORT  IOPORT4 /**< Rigth encoder A port.                */

#define R_ENCODER_A       PD2     /**< Right encoder A pin.                 */
#define R_ENCODER_B       PE3     /**< Right encoder B pin.                 */


/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

long encoderReadLeftCounter(void);
long encoderReadRightCounter(void);
long encoderReadLeftStateA(void);
long encoderReadLeftStateB(void);
long encoderReadRightStateA(void);
long encoderReadRightStateB(void);
void encoderInit(void);
void encoderGetWheelVelocity(void);

#endif /* IPENCODER_H */

