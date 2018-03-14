
/**
 *
 * @file    ip_encoder.h
 *
 * @brief   Encoder driver header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    27 August 2017
 *
 */

/*
    IP - Copyright (C) 2015..2018 Theodore Ateba

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef IP_ENCODER_H
#define IP_ENCODER_H

/**
 * @brief Encoders enumerations.
 */
typedef enum {
  ENCODER_L,          /**< Left encoder.                  */
  ENCODER_R,          /**< Right encoder.                 */
  ENCODER_L_STATE_A,  /**< State A of the left encoder.   */
  ENCODER_L_STATE_B,  /**< State B of the left encoder.   */
  ENCODER_R_STATE_A,  /**< State A of the right encoder.  */
  ENCODER_R_STATE_B   /**< State B of the right encoder.  */
}encoder_e;

/* Arduino Interruption */
#define INT2 2      /**< D19 [PD2], BLACK wire motor Right.                 */
#define INT3 3      /**< D18 [PD3], RED wire Motor Left.                    */

#define L_ENCODER_A_PORT  IOPORT4 /**< Left encoder A port.                 */
#define L_ENCODER_B_PORT  IOPORT7 /**< Left encoder B port.                 */

#define L_ENCODER_A       PD3     /**< Left encoder A pin.                  */
#define L_ENCODER_B       PG5     /**< Left encoder B pin.                  */

#define R_ENCODER_B_PORT  IOPORT5 /**< Right encoder B port.                */
#define R_ENCODER_A_PORT  IOPORT4 /**< Right encoder A port.                */

#define R_ENCODER_A       PD2     /**< Right encoder A pin.                 */
#define R_ENCODER_B       PE3     /**< Right encoder B pin.                 */

struct ENCODERDriver {
  uint8_t     id;       /**< Encoder identification name.                   */
  uint8_t     eichan;   /**< Encoder external interruption channel.         */
  ioportid_t  porta;    /**< Encoder port A.                                */
  uint8_t     pina;     /**< Encoder pin A.                                 */
  ioportid_t  portb;    /**< Encoder port B.                                */
  uint8_t     pinb;     /**< Encoder pin B.                                 */
  long        counter;  /**< Right encoder counter.                         */
  bool        statea;   /**< Left motor encoder A.                          */
  bool        stateb;   /**< Left motor encoder B.                          */
};

typedef struct ENCODERDriver ENCODERDriver;

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

bool encoder_left_read_state_a(void);
bool encoder_left_read_state_b(void);
bool encoder_right_read_state_a(void);
bool encoder_right_read_state_b(void);
void encoder_init(void);
void encoder_get_wheel_velocity(void);

#endif /* IP_ENCODER_H */

