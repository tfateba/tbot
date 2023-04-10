/*
    TBOT - Copyright (C) 2015...2023 Theodore Ateba

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
 * @file    mpu6050.c
 * @brief   Motion interface driver source file.
 *
 * @addtogroup MPU6050
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/*==========================================================================*/
/* Include files.                                                           */
/*==========================================================================*/

/* Project files. */
#include "i2c.hpp"
#include "motor.hpp"
#include "mpu6050.hpp"

I2c interfaceI2c;

/*==========================================================================*/
/* Driver functions.                                                        */
/*==========================================================================*/

/**
 * @brief   Get the identity of the MPU6050 device.
 *
 * @param[in] i2cp  pointer to the I2C driver interface
 * @param[in] idp   pointer to the id variable of the MPU6050 sensor
 *
 * @return    msg   result of the reading identity operation
 */
msg_t Mpu6050::getIdentity(I2CDriver *i2cp, uint8_t *idp) {

  uint8_t   txbuf;
  uint8_t   rxbuf;
  msg_t     msg;

  txbuf = MPU6050_WHO_AM_I;
  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, &txbuf, &rxbuf, 1);

  if (msg == MSG_OK) {
    *idp = rxbuf;
    return msg;
  }

  return msg;
}

/**
 * @brief   Put the MPU6050 in the Power Sleep Mode.
 *
 * @param[in] i2cp  pointer of the I2C driver interface
 *
 * @return    msg   result of the power mode configuration
 */
msg_t Mpu6050::sleep(I2CDriver *i2cp) {

  uint8_t config;
  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_PWR_MGMT_1;
  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, txbuf, &config, 1);

  if (msg != MSG_OK)
    return msg;

  txbuf[1] = config | (1 << 6);
  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 2);

  return msg;
}

/**
 * @brief   Remove the MPU6050 from the sleep mode.
 *
 * @param[in] i2cp  pointer of the i2C interface
 *
 * @return    msg   result of the wakeup operation
 */
msg_t Mpu6050::wakeup(I2CDriver *i2cp) {

  uint8_t config;
  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_PWR_MGMT_1;
  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, txbuf, &config, 1);

  if (msg != MSG_OK)
    return msg;

  txbuf[1] = config & ~(1 << 6);
  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 2);

  return msg;
}

/**
 * @brief   Reset the MPU6050.
 *
 * @param[in] i2cp  pointer of the i2C interface
 *
 * @return    msg   result of the reset operation
 */
msg_t Mpu6050::reset(I2CDriver *i2cp) {

  uint8_t config;
  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_PWR_MGMT_1;
  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, txbuf, &config, 1);

  if (msg != MSG_OK)
    return msg;

  txbuf[1] = config & ~(1 << 7);
  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 2);

  return msg;
}

/**
 * @brief   Configure the gyroscope full scale.
 *
 * @param[in] i2cp    pointer of the i2C interface
 * @param[in] scale   scale to set the MPU6050(250, 500, 1000 or 2000)
 *
 * @return    msg   result of the wakeup operation
 */
msg_t Mpu6050::configGyro(I2CDriver *i2cp, mpu6050_gyro_fs_e scale) {

  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_GYRO_CONFIG;

  if (scale == MPU6050_GYRO_FS_250) {
    txbuf[1] = txbuf[1] & ~(1 << 3);
    txbuf[1] = txbuf[1] & ~(1 << 4);
  }
  else if (scale == MPU6050_GYRO_FS_500) {
    txbuf[1] = txbuf[1] | (1 << 3);
    txbuf[1] = txbuf[1] & ~(1 << 4);
  }
  else if (scale == MPU6050_GYRO_FS_1000) {
    txbuf[1] = txbuf[1] & ~(1 << 3);
    txbuf[1] = txbuf[1] | (1 << 4);
  }
  else if (scale == MPU6050_GYRO_FS_2000) {
    txbuf[1] = txbuf[1] | (1 << 3);
    txbuf[1] = txbuf[1] | (1 << 4);
  }
  else
    return MSG_RESET;

  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 2);

  return msg;
}

/**
 * @brief   Configure the accelerometer full scale.
 *
 * @param[in] i2cp    pointer of the i2C interface
 * @param[in] scale   scale to set the MPU6050(2, 4, 8 or 16)
 *
 * @return    msg   result of the wakeup operation
 */
msg_t Mpu6050::configAccel(I2CDriver *i2cp, mpu6050_accel_fs_e scale) {

  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_ACCEL_CONFIG;

  if (scale == MPU6050_ACCEL_FS_2) {
    txbuf[1] = txbuf[1] & ~(1 << 3);
    txbuf[1] = txbuf[1] & ~(1 << 4);
  }
  else if (scale == MPU6050_ACCEL_FS_4) {
    txbuf[1] = txbuf[1] | (1 << 3);
    txbuf[1] = txbuf[1] & ~(1 << 4);
  }
  else if (scale == MPU6050_ACCEL_FS_8) {
    txbuf[1] = txbuf[1] & ~(1 << 3);
    txbuf[1] = txbuf[1] | (1 << 4);
  }
  else if (scale == MPU6050_ACCEL_FS_16) {
    txbuf[1] = txbuf[1] | (1 << 3);
    txbuf[1] = txbuf[1] | (1 << 4);
  }
  else
    return MSG_RESET;

  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 2);

  return msg;
}

/**
 * @brief   Get the gyroscope x axis offset.
 *
 * @param[in] i2cp  pointer of the i2C interface
 * @param[in] gxop  pointer to Gyroscope x axis offset
 *
 * @return    msg   result of the offset reading operation
 */
msg_t Mpu6050::getXGyroOffset(I2CDriver *i2cp, int16_t *gxop) {

  uint8_t txbuf;
  uint8_t rxbuf[2];
  msg_t   msg;

  txbuf = 0; /* TODO: Set the correct register. */
  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 2);
  *gxop = (int16_t) ((rxbuf[0] << 8) | rxbuf[1]);

  return msg;
}

/**
 * @brief   Set the gyroscope x axis offset.
 *
 * @param[in] i2cp    pointer of the i2C interface
 * @param[in] offset  gyroscope x axis offset
 *
 * @return    msg     result of the offset writing operation
 */
msg_t Mpu6050::setXGyroOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_X_GYRO_OFFSET_USRH;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;
  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 3);

  return msg;
}

/**
 * @brief   Get the gyroscope Y axis offset.
 *
 * @param[in] i2cp  pointer of the i2C interface
 * @param[in] gyop  gyroscope Y axis offset pointer
 *
 * @return    msg   result of the offset reading operation
 */
msg_t Mpu6050::getYGyroOffset(I2CDriver *i2cp, int16_t *gyop) {

  msg_t   msg;
  uint8_t txbuf;
  uint8_t rxbuf[2];

  txbuf = 0; /* TODO: Set the correct register. */
  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 2);
  *gyop = (int16_t) ((rxbuf[0] << 8) | rxbuf[1]);

  return msg;
}

/**
 * @brief   Set the gyroscope Y axis offset.
 *
 * @param[in] i2cp    pointer of the i2C interface
 * @param[in] offset  gyroscope Y axis offset
 *
 * @return    msg     result of the writing operation
 */
msg_t Mpu6050::setYGyroOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_Y_GYRO_OFFSET_USRH;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;
  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 3);

  return msg;
}

/**
 * @brief   Get the gyroscope Z axis offset.
 *
 * @param[in] i2cp  pointer of the i2C interface
 * @param[in] gzop  gyroscope Z axis offset pointer
 *
 * @return    msg   the result of the reading operation
 */
msg_t Mpu6050::getZGyroOffset(I2CDriver *i2cp, int16_t *gzop) {

  msg_t   msg;
  uint8_t txbuf;
  uint8_t rxbuf[2];

  txbuf = 0; /* TODO: Set the correct register. */
  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 2);

  *gzop = (int16_t) ((rxbuf[0] << 8) | rxbuf[1]);

  return msg;
}

/**
 * @brief   Set the gyroscope Z axis offset.
 *
 * @param[in] i2cp    pointer of the i2C interface
 * @param[in] offset  gyroscope Z axis offset
 *
 * @return    msg     the result of the writing operation
 */
msg_t Mpu6050::setZGyroOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_Z_GYRO_OFFSET_USRH;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;

  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 3);

  return msg;
}

/**
 * @brief   Set accelerometer Z axis offset.
 *
 * @param[in] i2cp    pointer of the i2C interface.
 * @param[in] offset  accelerometer Z axis offset.
 *
 * @return    msg     writing operation result.
 */
msg_t Mpu6050::setZAccelOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_Z_ACCEL_OFFSET_H;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;

  msg = interfaceI2c.write(i2cp, MPU6050_ADDR, txbuf, 3);

  return msg;
}

/**
 * @brief   Configure the accelerometer full scale.
 *
 * @param[in] i2cp  pointer of the i2C interface
 * @param[in] pmp   pointer to the power management configuration
 *
 * @return    msg   result of the reading  operation
 */
msg_t Mpu6050::read(I2CDriver *i2cp, uint8_t *pmp) {

  uint8_t config;
  uint8_t txbuf;
  msg_t   msg;

  txbuf = MPU6050_PWR_MGMT_1;

  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, &txbuf, &config, 1);

  if (msg != MSG_OK)
    return msg;
  *pmp = config;
  return msg;
}

/**
 * @brief   Read all axis of data and also the temperature.
 *
 * @param[in] i2cp    pointer of the i2C interface
 * @param[in] rxbuf   reception buffer for the i2c data readed
 *
 * @return    msg   result of the reading operation
 */
msg_t Mpu6050::readAllSensors(I2CDriver *i2cp, uint8_t *rxbuf) {

  uint8_t txbuf;
  msg_t   msg;

  txbuf = MPU6050_ACCEL_XOUT_H;

  msg = interfaceI2c.read(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 14);

  return msg;
}

/**
 * @brief   Read data from the IMU sensors.
 *
 * @param[in] i2cp  pointer of the i2C interface
 *
 * @return    msg   result of the reading operation
 */
msg_t Mpu6050::getData(I2CDriver *i2cp) {

  int16_t temp  = 0;
  int16_t x_a   = 0;
  int16_t y_a   = 0;
  int16_t z_a   = 0;
  int16_t x_g   = 0;
  int16_t y_g   = 0;
  int16_t z_g   = 0;
  uint8_t data[14];
  msg_t   msg;

  msg = readAllSensors(i2cp, data);

  if (msg != MSG_OK)
    msg = MSG_RESET;
  else {
    x_a |= (((int16_t)data[0]) << 8) | data[1];
    y_a |= (((int16_t)data[2]) << 8) | data[3];
    z_a |= (((int16_t)data[4]) << 8) | data[5];

    temp |= (((int16_t)data[6]) << 8)   | data[7];

    x_g |= (((int16_t)data[8]) << 8)  | data[9];
    y_g |= (((int16_t)data[10]) << 8) | data[11];
    z_g |= (((int16_t)data[12]) << 8) | data[13];

    temperature = (float)(temp/340)+36.53;

    x_accel = (float)x_a;
    y_accel = (float)y_a;
    z_accel = (float)z_a;

    x_gyro = (float)x_g;
    y_gyro = (float)y_g;
    z_gyro = (float)z_g;
  }

  return msg;
}

/**
 * @brief   Calibrate the MPU6050 accelerometer and gyroscope sensors.
 *
 * @param[in] i2cp  pointer of the i2C interface
 *
 * @return    msg   result of the calibration operation
 */
msg_t Mpu6050::doCalibration(I2CDriver *i2cp) {

  msg_t msg;
  uint16_t i;
  uint16_t sampleNumber = 1000;

  msg = setXGyroOffset(&I2CD1, 0);
  msg = setYGyroOffset(&I2CD1, 0);
  msg = setZGyroOffset(&I2CD1, 0);

  for (i = 0; i < sampleNumber + 100; i++) {
    msg = getData(i2cp);

    if (i > 100) {
      x_accel_offset  += x_accel;
      y_accel_offset  += y_accel;
      z_accel_offset  += z_accel;
      x_gyro_offset   += x_gyro;
      y_gyro_offset   += y_gyro;
      z_gyro_offset   += z_gyro;
    }
    chThdSleepMilliseconds(5);
  }

  x_accel_offset  = x_accel_offset/sampleNumber;
  y_accel_offset  = y_accel_offset/sampleNumber;
  z_accel_offset  = z_accel_offset/sampleNumber;
  x_gyro_offset   = x_gyro_offset/sampleNumber;
  y_gyro_offset   = y_gyro_offset/sampleNumber;
  z_gyro_offset   = z_gyro_offset/sampleNumber;

  msg = setXGyroOffset(&I2CD1, -(x_gyro_offset/4));
  msg = setYGyroOffset(&I2CD1, -(y_gyro_offset/4));
  msg = setZGyroOffset(&I2CD1, -(z_gyro_offset/4));

  return msg;
}

/**
 * @brief   Initialize the MPU6050 sensor.
 *
 * @param[in] i2cp  pointer of the i2C interface
 * @param[in] sad   mpu i2c serial address
 *
 * @return    msg   result of the initialization operation
 */
msg_t Mpu6050::init(I2CDriver *i2cp, mpu6050_sad_e sadVal) {

  msg_t msg;

  sad = sadVal;
  msg = reset(i2cp);
  msg = wakeup(i2cp);
  msg = configAccel(i2cp, MPU6050_ACCEL_FS_2);
  msg = configGyro(i2cp, MPU6050_GYRO_FS_250);

  return msg;
}

#ifdef __cplusplus
}
#endif

/** @} */
