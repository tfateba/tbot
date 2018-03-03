
/**
 *
 * @file    ip_conf.h
 *
 * @brief   Robot configuration header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    08 December 2016
 *
 */

#ifndef IP_CONF_H
#define IP_CONF_H

/*==========================================================================*/
/* Colour define for chprintf.                                              */
/*==========================================================================*/

#define chprintf_r(x, y)  chprintf(x, "\x1b[31m" y "\x1b[0m") /**< Red.     */
#define chprintf_g(x, y)  chprintf(x, "\x1b[32m" y "\x1b[0m") /**< Green.   */
#define chprintf_y(x, y)  chprintf(x, "\x1b[33m" y "\x1b[0m") /**< Yellow.  */
#define chprintf_b(x, y)  chprintf(x, "\x1b[34m" y "\x1b[0m") /**< Blue.    */
#define chprintf_m(x, y)  chprintf(x, "\x1b[35m" y "\x1b[0m") /**< Magenta. */
#define chprintf_c(x, y)  chprintf(x, "\x1b[36m" y "\x1b[0m") /**< Cyant.   */
#define chprintf_w(x, y)  chprintf(x, "\x1b[0m" y "\x1b[0m")  /**< White.   */

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

#define DEBUG     FALSE   /**< Debug activation in all source files.        */
#define DEBUG_MAI FALSE   /**< Debug activation in main file.               */
#define DEBUG_ASS FALSE   /**< Debug activation in asserv file              */
#define DEBUG_ENC FALSE   /**< Debug activation in asserv file              */
#define DEBUG_PID FALSE   /**< Debug activation in pid file.                */
#define DEBUG_MOT FALSE   /**< Debug activation in Motor file.              */
#define DEBUG_KAL FALSE   /**< Debug activation in Kalman filter file.      */
#define DEBUG_PWM FALSE   /**< Debug activation in PWM file.                */
#define DEBUG_I2C FALSE   /**< Debug activation in I2C file.                */
#define DABUG_MPU FALSE   /**< Debug activation in MPU6050 file.            */

#endif /* IP_CONF_H */

