
/**
 *
 * @file    ip_buzzer.h
 *
 * @brief   Buzzer header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    22 August 2017
 *
 */

#ifndef IP_BUZZER_H
#define IP_BUZZER_H

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

/* Pin used to control the buzzer. */
//#define BUZZER_PIN_PORT   IOPORT11
//#define BUZZER_PIN        46

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

void buzzerInit(void);
void buzzerSound(void);
void buzzerStopSound(void);

#endif /* IP_BUZZER_H */

