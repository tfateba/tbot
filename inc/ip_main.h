
/**
 *
 * @file    ip_main.h
 *
 * @brief   main application header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    17 November 2017
 *
 */

#ifndef IP_MAIN_H
#define IP_MAIN_H

/* Project files. */
#include "ip_encoder.h"
#include "ip_motor.h"
#include "ip_mpu6050.h"
#include "ip_pid.h"


struct ROBOTDriver {
  MPU6050Driver imu;

  PIDDriver     lpid;
  PIDDriver     rpid;

  MOTORDriver   lmotor;
  MOTORDriver   rmotor;

  ENCODERDriver lencoder;
  ENCODERDriver rencoder;
};

typedef struct ROBOTDriver ROBOTDriver;

#endif /* IP_MAIN_H */
