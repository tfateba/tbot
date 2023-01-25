/*
    TBOT - Copyright (C) 2015...2021 Theodore Ateba

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

/**
 * @file    encoder.h
 * @brief   Encoder driver header file.
 *
 * @addtogroup ENCODER
 * @{
 */

#ifndef ENCODER_H
#define ENCODER_H

/*==========================================================================*/
/* Includes files.                                                          */
/*==========================================================================*/

/*==========================================================================*/
/* Enumerations, Structures and macros.                                     */
/*==========================================================================*/

/* Arduino Interruption */
#define INT2 2      /**< ISR, D19 [PD2], BLACK wire motor Right.            */
#define INT3 3      /**< ISR, D18 [PD3], RED wire Motor Left.               */

#define L_ENCODER_PORT_A   IOPORT4  /**< Left encoder A port.               */
#define L_ENCODER_PORT_B   IOPORT7  /**< Left encoder B port.               */
#define L_ENCODER_PIN_A    PD3      /**< Left encoder A pin.                */
#define L_ENCODER_PIN_B    PG5      /**< Left encoder B pin.                */
#define L_ENCODER_EXT_INT  INT3     /**< Left encoder external isr pin.     */

#define R_ENCODER_PORT_A   IOPORT4  /**< Right encoder A port.              */
#define R_ENCODER_PORT_B   IOPORT5  /**< Right encoder B port.              */
#define R_ENCODER_PIN_A    PD2      /**< Right encoder A pin.               */
#define R_ENCODER_PIN_B    PE3      /**< Right encoder B pin.               */
#define R_ENCODER_EXT_INT  INT2     /**< Right encoder external isr pin.    */

/**
 * @brief Encoders identifier enumerations
 */
typedef enum {
  ENCODER_L,  /**< Left encoder.                                            */
  ENCODER_R,  /**< Right encoder.                                           */
} encoder_id_t;

/**
 * @brief   Encoder configuration structure.
 */
typedef struct {
  uint8_t     id;       /**< Encoder identification name.                   */
  uint8_t     eichan;   /**< Encoder external interruption channel.         */
  ioportid_t  porta;    /**< Encoder port A.                                */
  ioportid_t  portb;    /**< Encoder port B.                                */
  uint8_t     pina;     /**< Encoder pin A.                                 */
  uint8_t     pinb;     /**< Encoder pin B.                                 */
} ENCODERConfig;

/**
 * @brief   Encoder driver structure.
 */
typedef struct {
  ENCODERConfig config;
  volatile long          counter;  /**< Rigth encoder counter.                       */
  bool          statea;   /**< Left motor encoder A.                        */
  bool          stateb;   /**< Left motor encoder B.                        */
} ENCODERDriver;

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

bool encoderReadLeftStateA(void);
bool encoderReadLeftStateB(void);
bool encoderReadRightStateA(void);
bool encoderReadRightStateB(void);
//void encoderGetWheelVelocity(void);
void encoderInit(ENCODERDriver *edp, ENCODERConfig cfg);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */

/** @} */
