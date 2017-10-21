
/**
 *
 * @file    ipmpu6050.c
 *
 * @brief   Motion interface driver source file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    26 June 2016
 *
 */

/*==========================================================================*/
/* Include files.                                                           */
/*==========================================================================*/

/* Project local files. */
#include "ipi2c.h"
#include "ipmotor.h"
#include "ipmpu6050.h"

/*==========================================================================*/
/* Functions.                                                               */
/*==========================================================================*/

/**
 * @brief   Get the identity of the MPU6050 device.
 *
 * @param[in] i2cp  pointer to the I2C driver interface
 * @param[in] idp   pointer to the id variable of the MPU6050 sensor
 *
 * @return    msg   result of the reading identity operation
 */
msg_t mpu6050GetIdentity(I2CDriver *i2cp, uint8_t *idp) {

  uint8_t   txbuf;
  uint8_t   rxbuf;
  msg_t     msg;

  txbuf = MPU6050_WHO_AM_I;
  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, &txbuf, &rxbuf, 1);

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
msg_t mpu6050Sleep(I2CDriver *i2cp) {

  uint8_t config;
  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_PWR_MGMT_1;
  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, txbuf, &config, 1);

  if (msg != MSG_OK)
    return msg;

  txbuf[1] = config | (1 << 6);
  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 2);

  return msg;
}

/**
 * @brief   Remove the MPU6050 from the sleep mode.
 *
 * @param[in] i2cp  pointer of the i2C interface
 *
 * @return    msg   result of the wakeup operation
 */
msg_t mpu6050Wakeup(I2CDriver *i2cp) {

  uint8_t config;
  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_PWR_MGMT_1;
  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, txbuf, &config, 1);

  if (msg != MSG_OK)
    return msg;

  txbuf[1] = config & ~(1 << 6);
  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 2);

  return msg;
}

/**
 * @brief   Reset the MPU6050.
 *
 * @param[in] i2cp  pointer of the i2C interface
 *
 * @return    msg   result of the reset operation
 */
msg_t mpu6050Reset(I2CDriver *i2cp) {

  uint8_t config;
  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_PWR_MGMT_1;
  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, txbuf, &config, 1);

  if (msg != MSG_OK)
    return msg;

  txbuf[1] = config & ~(1 << 7);
  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 2);

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
msg_t mpu6050GyroConfig(I2CDriver *i2cp, mpu6050_gyro_fs_e scale) {

  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_GYRO_CONFIG;

  if (scale == MPU6050_GYRO_FS_250 ) {
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

  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 2);

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
msg_t mpu6050AccelConfig(I2CDriver *i2cp, mpu6050_accel_fs_e scale) {

  uint8_t txbuf[2];
  msg_t   msg;

  txbuf[0] = MPU6050_ACCEL_CONFIG;

  if (scale == MPU6050_ACCEL_FS_2 ) {
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

  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 2);

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
msg_t mpu6050GetXGyroOffset(I2CDriver *i2cp, int16_t *gxop) {

  uint8_t txbuf;
  uint8_t rxbuf[2];
  msg_t   msg;

  txbuf = 0; // TODO: Set the correct register.
  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 2);
  *gxop =(int16_t) ((rxbuf[0] << 8) | rxbuf[1]);

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
msg_t mpu6050SetXGyroOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_X_GYRO_OFFSET_USRH;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;
  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 3);

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
msg_t mpu6050GetYGyroOffset(I2CDriver *i2cp, int16_t *gyop) {

  msg_t   msg;
  uint8_t txbuf;
  uint8_t rxbuf[2];

  txbuf = 0; // TODO: Set the correct register.
  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 2);
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
msg_t mpu6050SetYGyroOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_Y_GYRO_OFFSET_USRH;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;
  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 3);

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
msg_t mpu6050GetZGyroOffset(I2CDriver *i2cp, int16_t *gzop) {

  msg_t   msg;
  uint8_t txbuf;
  uint8_t rxbuf[2];

  txbuf = 0; // TODO: Set the correct register.
  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 2);

  *gzop =(int16_t) ((rxbuf[0] << 8) | rxbuf[1]);

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
msg_t mpu6050SetZGyroOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_Z_GYRO_OFFSET_USRH;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;

  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 3);

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
msg_t mpu6050SetZAccelOffset(I2CDriver *i2cp, int16_t offset) {

  uint8_t txbuf[3];
  msg_t   msg;

  txbuf[0] = MPU6050_Z_ACCEL_OFFSET_H;
  txbuf[2] = offset;
  txbuf[1] = offset >> 8;

  msg = i2cWriteRegisters(i2cp, MPU6050_ADDR, txbuf, 3);

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
msg_t mpu6050Read(I2CDriver *i2cp, uint8_t *pmp) {

  uint8_t config;
  uint8_t txbuf;
  msg_t   msg;

  txbuf = MPU6050_PWR_MGMT_1;

  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, &txbuf, &config, 1);

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
msg_t mpu6050ReadAllSensors(I2CDriver *i2cp, uint8_t *rxbuf) {

  uint8_t txbuf;
  msg_t   msg;

  txbuf = MPU6050_ACCEL_XOUT_H;

  msg = i2cReadRegisters(i2cp, MPU6050_ADDR, &txbuf, rxbuf, 14);

  return msg;
}

/**
 * @brief   Read data from the IMU sensors.
 *
 * @param[in] i2cp  pointer of the i2C interface
 * @param[in] mpup  pointer of the mpu6050 data structure
 *
 * @return    msg   result of the reading operation
 */
msg_t mpu6050GetData(I2CDriver *i2cp, mpu6050_t *mpup) {

  int16_t temp    = 0;
  int16_t x_accel = 0;
  int16_t y_accel = 0;
  int16_t z_accel = 0;
  int16_t x_gyro  = 0;
  int16_t y_gyro  = 0;
  int16_t z_gyro  = 0;
  uint8_t mpuData[14];
  msg_t   msg;

  msg = mpu6050ReadAllSensors(i2cp, mpuData);

  if (msg != MSG_OK)
    return MSG_RESET;
  else {
    x_accel |= (((int16_t)mpuData[0]) << 8) | mpuData[1];
    y_accel |= (((int16_t)mpuData[2]) << 8) | mpuData[3];
    z_accel |= (((int16_t)mpuData[4]) << 8) | mpuData[5];

    temp |= (((int16_t)mpuData[6]) << 8)   | mpuData[7];

    x_gyro |= (((int16_t)mpuData[8]) << 8)  | mpuData[9];
    y_gyro |= (((int16_t)mpuData[10]) << 8) | mpuData[11];
    z_gyro |= (((int16_t)mpuData[12]) << 8) | mpuData[13];

    mpup->temp = (float)(temp/340)+36.53;

    mpup->x_accel = (float)x_accel;
    mpup->y_accel = (float)y_accel;
    mpup->z_accel = (float)z_accel;

    mpup->x_gyro = (float)x_gyro;
    mpup->y_gyro = (float)y_gyro;
    mpup->z_gyro = (float)z_gyro;

    return msg;
  }
}

/**
 * @brief   Calibrate the MPU6050 accelerometer and gyroscope sensors.
 *
 * @param[in] i2cp  pointer of the i2C interface
 * @param[in] mpup  pointer of the mpu data structure
 *
 * @return    msg   result of the calibration operation
 */
msg_t mpu6050Calibration(I2CDriver *i2cp, mpu6050_t *mpup) {

  msg_t msg;
  uint16_t i;
  uint16_t sampleNumber = 1000;

  msg = mpu6050SetXGyroOffset(&I2CD1, 0);
  msg = mpu6050SetYGyroOffset(&I2CD1, 0);
  msg = mpu6050SetZGyroOffset(&I2CD1, 0);

  for (i = 0; i < sampleNumber + 100; i++) {
    msg = mpu6050GetData(i2cp, mpup);

    if (i > 100) {
      mpup->x_accel_offset  += mpup->x_accel;
      mpup->y_accel_offset  += mpup->y_accel;
      mpup->z_accel_offset  += mpup->z_accel;
      mpup->x_gyro_offset   += mpup->x_gyro;
      mpup->y_gyro_offset   += mpup->y_gyro;
      mpup->z_gyro_offset   += mpup->z_gyro;
    }
    chThdSleepMilliseconds(5);
  }

  mpup->x_accel_offset  = mpup->x_accel_offset/sampleNumber;
  mpup->y_accel_offset  = mpup->y_accel_offset/sampleNumber;
  mpup->z_accel_offset  = mpup->z_accel_offset/sampleNumber;
  mpup->x_gyro_offset   = mpup->x_gyro_offset/sampleNumber;
  mpup->y_gyro_offset   = mpup->y_gyro_offset/sampleNumber;
  mpup->z_gyro_offset   = mpup->z_gyro_offset/sampleNumber;

  msg = mpu6050SetXGyroOffset(&I2CD1, -(mpup->x_gyro_offset/4)); // TODO: Utiliser un decalage de deux ici pour faire la division.
  msg = mpu6050SetYGyroOffset(&I2CD1, -(mpup->y_gyro_offset/4));
  msg = mpu6050SetZGyroOffset(&I2CD1, -(mpup->z_gyro_offset/4));

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
msg_t mpu6050Init(I2CDriver *i2cp, mpu6050_t *mpu, mpu6050_sad_e sad) {

  msg_t msg;

  mpu->sad = sad;
  msg = mpu6050Reset(i2cp);
  msg = mpu6050Wakeup(i2cp);
  msg = mpu6050AccelConfig(i2cp, MPU6050_ACCEL_FS_2);
  msg = mpu6050GyroConfig(i2cp, MPU6050_GYRO_FS_250);

  return msg;
}

