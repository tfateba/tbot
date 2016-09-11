/**
 *
 * @file    sam_motor.c
 *
 * @brief   Fonctions use to manage the LM298N dual step motor driver.
 *
 * @author  Theodore Ateba
 *
 * @date    27 June 2016
 *TODO: Correst the description with the LM298N driver
 * @description The robot is an autonomous explorer robot. It avoid obstacle
 *              and run in stand-alone mode. To do that, the robot use a 
 *              ultrasonic sensor to mesure the distance.
 *
 *              The motor driver is the HG7881 dual channel.
 *              We can control two motors with this driver.
 *              The interface is as follow:
 *              B-IA  Motor B input A. Speed control with a PWM.
 *              B-IB  Motor B input B. Direction control with a GPIO.
 *              GND   Ground.
 *              VCC   Operating voltage 2,5-12V.
 *              A-IA  Motor A input A. Speed control with a PWM.
 *              A-IB  Motor A input B. Direction control with a GPIO.
 *
 *              The Driver wiring is made as follow:
 *              B-IA pin  GPIOA8 pin
 *              B-IB pin  GPIOB10 pin
 *              GND pin   GND pin
 *              VCC pin   5V pin
 *              A-IA pin  GPIOA9 pin
 *              A-IB pin  GPIOC7 pin
 *
 *              The Ultrasonic sensor is connected as follow:
 *              VCC pin   5V
 *              Trig pin  GPIOB5
 *              Echo pin  GPIOB4
 *              GND pin   GND pin
 *
 *
 * @description The Motor driver is the L298N from ST Micro-electronics
 *              It is a Dual Stepper Motors Driver.
 *              Driver-Pin  Pin-type  Nucleo-Pin  Description
 *                  IN1       GPIO      GPIOA1    Motor 1 direction control.
 *                  IN2       GPIO      GPIOA1    Motor 1 direction control.
 *                  ENA       PWM       GPIOA1    Motor 1 speed control.
 *                  IN3       GPIO      GPIOA1    Motor 2 direction control.
 *                  IN4       GPIO      GPIOA1    Motor 2 direction control.
 *                  ENB       PWM       GPIOA1    Motor 2 speed control.
 *                  VCC       Power       5V      Driver power supply.
 *                  GND       Power       GND     Driver power supply.
 */

/*===========================================================================*/
/* Includes Files                                                            */
/*===========================================================================*/
#include "sam_motor.h"

/*===========================================================================*/
/* Global variables, I2C TX and RX buffers, I2C and Serial Configurations    */
/*===========================================================================*/

/*===========================================================================*/
/* Functions and threads                                                     */
/*===========================================================================*/

/**
 * @fn    motor_setDirection 
 * @brief Set the direction of a motor, forward or backward
 *
 * @param[in] motorId   The Id of the motor top be control.
 * @param[in] direction The direction of the selected motor.
 */
void motor_setDirection(uint8_t motorId, bool direction){
  if(motorId == MOTOR1){
    if(direction ==  MOTOR_FORWARD){
      palSetPad(LM298N_IN1_PORT, LM298N_IN1_PIN);
      palClearPad(LM298N_IN2_PORT, LM298N_IN2_PIN);
    }
    else if(direction == MOTOR_REVERSE){
      palSetPad(LM298N_IN2_PORT, LM298N_IN2_PIN);
      palClearPad(LM298N_IN1_PORT, LM298N_IN1_PIN);
    }
  }
  else if(motorId == MOTOR2){
    if(direction ==  MOTOR_FORWARD){
      palSetPad(LM298N_IN3_PORT, LM298N_IN3_PIN);
      palClearPad(LM298N_IN4_PORT, LM298N_IN4_PIN);
    }
    else if(direction == MOTOR_REVERSE){
      palSetPad(LM298N_IN4_PORT, LM298N_IN4_PIN);
      palClearPad(LM298N_IN3_PORT, LM298N_IN3_PIN);
    }
  }
}

/**
 * @fn    motor_setSpeed
 * @brief Set the motor speed
 *
 * @param[in] pwmp    PWM driver interface pointer
 * @param[in] motorId Motor to control, ID = 1 or 2 for this project.
 * @param[in] speed   The speed to driver the motor.
 */
void motor_setSpeed(PWMDriver *pwmp, uint8_t motorId, uint16_t speed){
  if(motorId == MOTOR1)
    pwm_setPulseWidth(pwmp, MOTOR1_PWM_CHANNEL, speed);
  else if(motorId == MOTOR2)
    pwm_setPulseWidth(pwmp, MOTOR2_PWM_CHANNEL, speed);
}

/**
 * @fn    motor_init
 * @brief Initialize the motors. Direction and configure the pwm for speed.
 *
 * @param[in] pwmp    The pwm interface pointer
 * @param[in] pwmcfg  The pwm interface configuration
 */
void motors_init(PWMDriver *pwmp, PWMConfig *pwmcfg){
  /* Configure the motors pins */
  palSetPadMode(LM298N_IN1_PORT, LM298N_IN1_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LM298N_IN2_PORT, LM298N_IN2_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LM298N_IN3_PORT, LM298N_IN3_PIN, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(LM298N_IN4_PORT, LM298N_IN4_PIN, PAL_MODE_OUTPUT_PUSHPULL);

  /* Set the direction of the motors */
  motor_setDirection(MOTOR1, MOTOR_FORWARD);
  motor_setDirection(MOTOR2, MOTOR_FORWARD);
  
  /* Configure the motors PWM pin for speed control */
  pwm_init(pwmp, pwmcfg);
}

/**
 * @fn    demo
 * @brief demo function to test the motor.
 *
 * @param[in] pwmp    The pwm interface pointer
 * @param[in] pwmcfg  The pwm interface configuration
 */
void demo(PWMDriver *pwmp, PWMConfig *pwmcfg){
  uint16_t i;

  pwm_disable(pwmp);
  chThdSleepMilliseconds(1000);
  pwm_enable(pwmp, pwmcfg, 0);
  pwm_enable(pwmp, pwmcfg, 1);
  chThdSleepMilliseconds(1000);

  motor_setDirection(MOTOR1, MOTOR_FORWARD);
  motor_setDirection(MOTOR2, MOTOR_FORWARD);
  for(i = 0; i <= 10000; i+= 1000){
    motor_setSpeed(pwmp, MOTOR1, i);
    motor_setSpeed(pwmp, MOTOR2, i);
    chThdSleepMilliseconds(100);
  }
  for(i = 10000; i > 0; i-= 1000){
    motor_setSpeed(pwmp, MOTOR1, i);
    motor_setSpeed(pwmp, MOTOR2, i);
    chThdSleepMilliseconds(100);
  }

  motor_setDirection(MOTOR1, MOTOR_REVERSE);
  motor_setDirection(MOTOR2, MOTOR_REVERSE);
  for(i = 0; i <= 10000; i+= 1000){
    motor_setSpeed(pwmp, MOTOR1, i);
    motor_setSpeed(pwmp, MOTOR2, i);
    chThdSleepMilliseconds(100);
  }
  for(i = 10000; i > 0; i-= 1000){
    motor_setSpeed(pwmp, MOTOR1, i);
    motor_setSpeed(pwmp, MOTOR2, i);
    chThdSleepMilliseconds(100);
  }
  pwm_disable(pwmp);
}

/**
 * @fn    controlMotor
 * @brief Control the motors of the robot.
 *
 * @param[in] pwmp    The pwm interface pointer.
 * @param[in] pwmcfg  The pwm interface configuration.
 */
void controlMotor(PWMDriver *pwmp, float speed){
/*
  // Set the direction
  if( speed < 0 ){
    motor_setDirection(MOTOR1, MOTOR_FORWARD);
    motor_setDirection(MOTOR2, MOTOR_FORWARD);
  }
  else{
    motor_setDirection(MOTOR1, MOTOR_REVERSE);
    motor_setDirection(MOTOR2, MOTOR_REVERSE);
  }
*/
  // Set speed
  motor_setSpeed(pwmp, MOTOR1, speed);
  motor_setSpeed(pwmp, MOTOR2, speed);
}
