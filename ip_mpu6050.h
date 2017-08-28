
/**
 *
 * @file    ip_mpu6050.h
 *
 * @brief   Motion interface driver header file.
 *
 * @author  Theodore Ateba, tf.ateba@gmail.com
 *
 * @date    22 June 2016
 *
 */

#ifndef IP_MPU6050_H
#define IP_MPU6050_H

/*==========================================================================*/
/* Include file.                                                            */
/*==========================================================================*/

/*==========================================================================*/
/* Structures and enumerations.                                             */
/*==========================================================================*/

/**
 * @brief   MPU6050 Slave I2C adress enumeration type.
 */
typedef enum {
  MPU6050_ADDR = 0x68  /**< I2C adress of the mpu6050 module.               */
}mpu6050_sad_e;

/**
 * @brief   External Frame Synchronisation values
 */
typedef enum {
  MPU6050_INPUT_DISABLED  = 0x00, /**< External Frame Synchronization 0.    */
  MPU6050_TEMP_OUT_L      = 0x01, /**< External Frame Synchronization 1.    */
  MPU6050_GYRO_XOUT_L     = 0x02, /**< External Frame Synchronization 2.    */
  MPU6050_GYRO_YOUT_L     = 0x03, /**< External Frame Synchronization 3.    */
  MPU6050_GYRO_ZOUT_L     = 0x04, /**< External Frame Synchronization 4.    */
  MPU6050_ACCEL_XOUT_L    = 0x05, /**< External Frame Synchronization 5.    */
  MPU6050_ACCEL_YOUT_L    = 0x06, /**< External Frame Synchronization 6.    */
  MPU6050_ACCEL_ZOUT_L    = 0x07  /**< External Frame Synchronization 7.    */
}mpu6050_ext_sync_set_e;          /**< External Frame Synchronization type. */

/**
 * @brief   Digital Low Pass Filter setting for Accelerator and the Gyroscope.
 */
typedef enum {
  MPU6050_DLPF_CFG_0  = 0x00, /**< Digital Low Pass Filter value 0.         */
  MPU6050_DLPF_CFG_1  = 0x01, /**< Digital Low Pass Filter value 1.         */
  MPU6050_GLPF_CFG_2  = 0x02, /**< DIgital Low Pass Filter value 2.         */
  MPU6050_DLPF_CFG_3  = 0x03, /**< Digital Low Pass Filter value 3.         */
  MPU6050_DLPF_CFG_4  = 0x04, /**< Digital Low Pass Filter value 4.         */
  MPU6050_DLPF_CFG_5  = 0x05, /**< Digital Low Pass Filetr value 5.         */
  MPU6050_DLPF_CFG_6  = 0x06, /**< Digital Low Pass Filter value 6.         */
  MPU6050_DLPF_CFG_7  = 0x07  /**< Digital Low Pass Filter value 7.         */
}mpu6050_dlpf_cfg_e;          /**< Digital Low Pass Filter type             */

/**
 * @brief   Power management 1 enumeration. Configuration of the device
 *          Power mode and Clock source.
 */
typedef enum {
  MPU6050_PM1_DEVICE_RESET  = 0x00, /**< Bit for reset the entire device.   */
  MPU6050_PM1_SLEEP         = 0x01, /**< Bit for power sleep mode.          */
  MPU6050_PM1_CYCLE         = 0x02, /**< Bit for Cycle mode.                */
  MPU6050_PM1_TEMP_DIS      = 0x03, /**< Bit for disabling temp sensor.     */
  MPU6050_PM1_CLKSEL        = 0x04  /**< Bit for device clock source.       */
}mpu6050_pm1_e;                     /**< Power Management type enumeration. */

/**
 * @brief   Power Management 2 enumeration. Configuration of the device
 *          Acceleration frequency wake-up. Also help to enable or disable
 *          individual accelerometer and gyroscope axes.
 */
typedef enum {
  MPU6050_LP_WAKE_CTRL  = 0x00, /**< Bits to control Frequency of wake-up   */
  MPU6050_STBY_XA       = 0x01, /**< Bit to control X axis accelerometer.   */
  MPU6050_STBY_YA       = 0x02, /**< Bit to control Y axis accelerometer.   */
  MPU6050_STBY_ZA       = 0x03, /**< Bit to control Z axis accelerometer.   */
  MPU6050_STBY_XG       = 0x04, /**< Bit to control X axis gyroscope.       */
  MPU6050_STBY_YG       = 0x05, /**< Bit to control Y axis gyroscope.       */
  MPU6050_STBY_ZG       = 0x06  /**< Bit to control Z axis gyroscope.       */
}mpu6050_pm2_e;

/**
 * @brief   Gyroscope full scale values.
 */
typedef enum {
  MPU6050_GYRO_FS_250   = 0x00, /**< Gyroscope full Scale value 0.          */
  MPU6050_GYRO_FS_500   = 0x01, /**< Gyroscope full Scale value 1.          */
  MPU6050_GYRO_FS_1000  = 0x02, /**< Gyroscope full Scale value 2.          */
  MPU6050_GYRO_FS_2000  = 0x03  /**< Gyroscope full Scale value 3.          */
}mpu6050_gyro_fs_e;             /**< Gyroscope full scale enum.             */

/**
 * @brief   Accelerometer full scale values.
 */
typedef enum {
  MPU6050_ACCEL_FS_2  = 0x00, /**< Accelerometer full scale value 0.        */
  MPU6050_ACCEL_FS_4  = 0x01, /**< Accelerometer full scale value 1.        */
  MPU6050_ACCEL_FS_8  = 0x02, /**< Accelerometer full scale value 2.        */
  MPU6050_ACCEL_FS_16 = 0x03  /**< Accelerometer full scale value 3.        */
}mpu6050_accel_fs_e;          /**< Accelerometer full scale enum.           */

/**
 * @brief   MPU6050 data structure
 */
typedef struct {
  mpu6050_sad_e sad;            /**< MPU6050 I2C slave address              */
  float         x_accel;        /**< Accelerometer x data.                  */
  float         y_accel;        /**< Accelerometer y data.                  */
  float         z_accel;        /**< Accelerometer z data.                  */
  float         x_accel_offset; /**< Accelerometer x data.                  */
  float         y_accel_offset; /**< Accelerometer y data.                  */
  float         z_accel_offset; /**< Accelerometer z data.                  */
  float         x_gyro;         /**< Gyroscope x data.                      */
  float         y_gyro;         /**< Gyroscope y data.                      */
  float         z_gyro;         /**< Gyroscope z data.                      */
  float         x_gyro_offset;  /**< Gyroscope x data.                      */
  float         y_gyro_offset;  /**< Gyroscope y data.                      */
  float         z_gyro_offset;  /**< Gyroscope z data.                      */
  float         pitch;          /**< MPU Pitch angle.                       */
  float         roll;           /**< MPU Roll angle.                        */
  float         yaw;            /**< MPU Yaw angle.                         */
  float         pitch_k;        /**< Filtered Pitch angle by Kalmman filter.*/
  float         roll_k;         /**< Filtered roll angle by kalman filter.  */
  float         yaw_k;          /**< Filtered yaw angle by kalman filter.   */
  float         pitch_c;        /**< Pitch angle by complementary filter.   */
  float         roll_c;         /**< Roll angle by complementary filter.    */
  float         yaw_c;          /**< Yaw angle by complementary filter.     */
  float         temp;           /**< MPU Temperature.                       */
}mpu6050_t;

/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/* MPU6050 Registers List */
#define MPU6050_XG_OFFSET_TC        0x00 /**< TODO:.                        */
#define MPU6050_YG_OFFSET_TC        0x01
#define MPU6050_ZG_OFFSET_TC        0x02
#define MPU6050_X_FINE_GAIN         0x03
#define MPU6050_Y_FINE_GAIN         0x04
#define MPU6050_Z_FINE_GAIN         0x05
#define MPU6050_X_ACCEL_OFFSET_H    0x06
#define MPU6050_X_ACCEL_OFFSET_L    0x07
#define MPU6050_Y_ACCEL_OFFSET_H    0x08
#define MPU6050_Y_ACCEL_OFFSET_L    0x09
#define MPU6050_Z_ACCEL_OFFSET_H    0x0A
#define MPU6050_Z_ACCEL_OFFSET_L    0x0B

#define MPU6050_SELF_TEST_X         0x0D /**< Self Test Register.           */
#define MPU6050_SELF_TEST_Y         0x0E /**< Self Test Register.           */
#define MPU6050_SELF_TEST_Z         0x0F /**< Self Test Register.           */
#define MPU6050_SELF_TEST_A         0x10 /**< Self Test Register.           */

#define MPU6050_X_GYRO_OFFSET_USRH  0x13
#define MPU6050_X_GYRO_OFFSET_USRL  0x14
#define MPU6050_Y_GYRO_OFFSET_USRH  0x15
#define MPU6050_Y_GYRO_OFFSET_USRL  0x16
#define MPU6050_Z_GYRO_OFFSET_USRH  0x17
#define MPU6050_Z_GYRO_OFFSET_USRL  0x18

#define MPU6050_SMPLRT_DIV          0x19 /**< Sample Rate Divider.          */
#define MPU6050_CONFIG              0x1A /**< Configuration.                */
#define MPU6050_GYRO_CONFIG         0x1B /**< Gyroscope Config.             */
#define MPU6050_ACCEL_CONFIG        0x1C /**< Accelerometer Config.         */

#define MPU6050_FF_THR              0x1D
#define MPU6050_FF8DUR              0x1E
#define MPU6050_MOT_THR             0x1F /**< Motion Det Threshold.         */
#define MPU6050_MOT_DUR             0x20
#define MPU6050_ZRMOT               0x21
#define MPU6050_ZRDUR               0x22

#define MPU6050_FIFO_EN             0x23 /**< Fifo Enable.                  */
#define MPU6050_I2C_MST_CTRL        0x24 /**< I2C Master Control.           */
#define MPU6050_I2C_SLV0_ADDR       0x25 /**< I2C Slave 0 Control.          */
#define MPU6050_I2C_SLV0_REG        0x26 /**< I2C Slave 0 Control.          */
#define MPU6050_I2C_SLV0_CTRL       0x27 /**< I2C Slave 0 Control.          */
#define MPU6050_I2C_SLV1_ADDR       0x28 /**< I2C SLave 1 Control.          */
#define MPU6050_I2C_SLV1_REG        0x29 /**< I2C Slave 1 Control.          */
#define MPU6050_I2C_SLV1_CTRL       0x2A /**< I2C Slave 1 Control.          */
#define MPU6050_I2C_SLV2_ADDR       0x2B /**< I2C Slave 2 Control.          */
#define MPU6050_I2C_SLV2_REG        0x2C /**< I2C Slave 2 Control.          */
#define MPU6050_I2C_SLV2_CTRL       0x2D /**< I2C Slave 2 Control.          */
#define MPU6050_I2C_SLV3_ADDR       0x2E /**< I2C Slave 3 Control.          */
#define MPU6050_I2C_SLV3_REG        0x2F /**< I2C Slave 3 Control.          */
#define MPU6050_I2C_SLV3_CTRL       0x30 /**< I2C Slave 3 Control.          */
#define MPU6050_I2C_SLV4_ADDR       0x31 /**< I2C Slave 4 Control.          */
#define MPU6050_I2C_SLV4_REG        0x32 /**< I2C Slave 4 Control.          */
#define MPU6050_I2C_SLV4_DO         0x33 /**< I2C Slave 4 Control.          */
#define MPU6050_I2C_SLV4_CTRL       0x34 /**< I2C Slave 4 Control.          */
#define MPU6050_I2C_SLV4_DI         0x35 /**< I2C Slave 4 Control.          */
#define MPU6050_I2C_MST_STATUS      0x36 /**< I2C Master Status.            */
#define MPU6050_INT_PIN_CFG         0x37 /**< INT Pin Enable Config.        */
#define MPU6050_INT_ENABLE          0x38 /**< Interrupt Enable.             */
#define MPU6050_DMP_INT_STATUS      0x39
#define MPU6050_INT_STATUS          0x3A /**< Interrupt Status.             */
#define MPU6050_ACCEL_XOUT_H        0x3B /**< Accel Measurements.           */
#define MPU6050_ACCEL_XOUT_L        0x3C /**< Accel Measurements.           */
#define MPU6050_ACCEL_YOUT_H        0x3D /**< Accel Measurements.           */
#define MPU6050_ACCEL_YOUT_L        0x3E /**< Accel Measurements.           */
#define MPU6050_ACCEL_ZOUT_H        0x3F /**< Accel Measurements.           */
#define MPU6050_ACCEL_ZOUT_L        0x40 /**< Accel Measurements.           */
#define MPU6050_TEMP_OUT_H          0x41 /**< Temp Measurements.            */
#define MPU6050_TEMP_OUT_L          0x42 /**< Temp Measurements.            */
#define MPU6050_GYRO_XOUT_H         0x43 /**< Gyro Measurements.            */
#define MPU6050_GYRO_XOUT_L         0x44 /**< Gyro Measurements.            */
#define MPU6050_GYRO_YOUT_H         0x45 /**< Gyro Measurements.            */
#define MPU6050_GYRO_YOUT_L         0x46 /**< Gyro Measurements.            */
#define MPU6050_GYRO_ZOUT_H         0x47 /**< Gyro Measurements.            */
#define MPU6050_GYRO_ZOUT_L         0x48 /**< Gyro Measurements.            */
#define MPU6050_EXT_SENS_DATA_00    0x49 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_01    0x4A /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_02    0x4B /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_03    0x4C /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_04    0x4D /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_05    0x4E /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_06    0x4F /**< Ecternal Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_07    0x50 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_08    0x51 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_09    0x52 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_10    0x53 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_11    0x54 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_12    0x55 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_13    0x56 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_14    0x57 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_15    0x58 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_16    0x59 /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_17    0x5A /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_18    0x5B /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_19    0x5C /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_20    0x5D /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_21    0x5E /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_22    0x5F /**< External Sensor Data.         */
#define MPU6050_EXT_SENS_DATA_23    0x60 /**< External Sensor Data.         */
#define MPU6050_MOT_DETECT_STATUS   0x61
#define MPU6050_I2C_SLV0_DO         0x63 /**< I2C Slave 0 Data Out.         */
#define MPU6050_I2C_SLV1_DO         0x64 /**< I2C Slave 1 Data Out.         */
#define MPU6050_I2C_SLV2_DO         0x65 /**< I2C Slave 2 Data Out.         */
#define MPU6050_I2C_SLV3_DO         0x66 /**< I2C Slave 3 Data Out.         */
#define MPU6050_MST_DELAY_CTRL      0x67 /**< I2C Master Delay CTRL         */
#define MPU6050_SIGNAL_PATH_RESET   0x68 /**< Signal Path Reset.            */
#define MPU6050_MOT_DETECT_CTRL     0x69 /**< Motion Det Control.           */
#define MPU6050_USER_CTRL           0x6A /**< User Control.                 */
#define MPU6050_PWR_MGMT_1          0x6B /**< Power Management 1.           */
#define MPU6050_PWR_MGMT_2          0x6C /**< Power Management 2.           */
#define MPU6050_BANK_SEL            0x6D
#define MPU6050_MEM_START_ADDR      0x6E
#define MPU6050_MEM_R_W             0x6F
#define MPU6050_DMP_CONFIG_1        0x70
#define MPU6050_DMP_CONFIG_2        0x71
#define MPU6050_FIFO_COUNTH         0x72 /**< Fifo Count Register.          */
#define MPU6050_FIFO_COUNTL         0x73 /**< Fifo Count Register.          */
#define MPU6050_FIFO_R_W            0x74 /**< Fifo Read Write.              */
#define MPU6050_WHO_AM_I            0x75 /**< Who Am I.                     */

/*==========================================================================*/
/* Functions prototypes.                                                    */
/*==========================================================================*/

msg_t mpu6050GetIdentity(I2CDriver *i2cp, uint8_t *idp);
msg_t mpu6050Sleep(I2CDriver *i2cp);
msg_t mpu6050Wakeup(I2CDriver *i2cp);
msg_t mpu6050Reset(I2CDriver *i2cp);
msg_t mpu6050GyroConfig(I2CDriver *i2cp, mpu6050_gyro_fs_e scale);
msg_t mpu6050AccelConfig(I2CDriver *i2cp, mpu6050_accel_fs_e scale);
msg_t mpu6050Read(I2CDriver *i2cp, uint8_t *pmp);
msg_t mpu6050ReadAllSensors(I2CDriver *i2cp, uint8_t *rxbuf);
msg_t mpu6050SetXGyroOffset(I2CDriver *i2cp, int16_t offset);
msg_t mpu6050SetYGyroOffset(I2CDriver *i2cp, int16_t offset);
msg_t mpu6050SetZGyroOffset(I2CDriver *i2cp, int16_t offset);
msg_t mpu6050SetZAccelOffset(I2CDriver *i2cp, int16_t offset);
msg_t mpu6050GetData(I2CDriver *i2cp, mpu6050_t *mpu);
msg_t mpu6050Calibration(I2CDriver *i2cp, mpu6050_t *mpu);
msg_t mpu6050Init(I2CDriver *i2cp, mpu6050_t *mpu, mpu6050_sad_e sad);

#endif /* IP_MPU6050_H */

