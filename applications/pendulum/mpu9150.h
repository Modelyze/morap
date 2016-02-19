/*
 * File:   pic32_i2c.h
 * Author: Viktor Kozma
 *
 * Defines functions for reading and filtering
 * signals from an MPU9150 imu
 *
 * Created on May 7, 2015
 */

#ifndef _MPU9150_H
#define	_MPU9150_H

// Addresses
#define MPU9150_I2C_ADDRESS 0x68

// MPU9150 registers
#define MPU9150_SMPLRT_DIV      0x19
#define MPU9150_CONFIG          0x1A
#define MPU9150_GYRO_CONFIG     0x1B
#define MPU9150_ACCEL_CONFIG    0x1C
#define MPU9150_FIFO_EN         0x23
#define MPU9150_I2C_MST_CTRL    0x24
#define MPU9150_I2C_MS_STATUS   0x36
#define MPU9150_INT_PIN_CFG     0x37
#define MPU9150_INT_ENABLE      0x38
#define MPU9150_INT_STATUS      0x3A

#define MPU9150_ACCEL_XOUT_H    0x3B
#define MPU9150_ACCEL_XOUT_L    0x3C
#define MPU9150_ACCEL_YOUT_H    0x3D
#define MPU9150_ACCEL_YOUT_L    0x3E
#define MPU9150_ACCEL_ZOUT_H    0x3F
#define MPU9150_ACCEL_ZOUT_L    0x40
#define MPU9150_TEMP_OUT_H      0x41
#define MPU9150_TEMP_OUT_L      0x42
#define MPU9150_GYRO_XOUT_H     0x43
#define MPU9150_GYRO_XOUT_L     0x44
#define MPU9150_GYRO_YOUT_H     0x45
#define MPU9150_GYRO_YOUT_L     0x46
#define MPU9150_GYRO_ZOUT_H     0x47
#define MPU9150_GYRO_ZOUT_L     0x48

#define MPU9150_USER_CTRL       0x6A
#define MPU9150_PWR_MGMT_1      0x6B
#define MPU9150_PWR_MGMT_2      0x6C

#define MPU9150_FIFO_COUNTH     0x72
#define MPU9150_FIFO_COUNTL     0x73
#define MPU9150_FIFO_R_W        0x74
#define MPU9150_WHO_AM_I        0x75

// AK8975C magnetometer
#define AK8975C_I2C_ADDRESS     0xC

// Registers for magnetometer
#define AK8975C_WIA             0x00
#define AK8975C_INFO            0x01
#define AK8975C_ST1             0x02
#define AK8975C_HXL             0x03
#define AK8975C_HXH             0x04
#define AK8975C_HYL             0x05
#define AK8975C_HYH             0x06
#define AK8975C_HZL             0x07
#define AK8975C_HZH             0x08
#define AK8975C_ST2             0x09
#define AK8975C_CNTL            0x0A
#define AK8975C_ASAX            0x10
#define AK8975C_ASAY            0x11
#define AK8975C_ASAZ            0x12


/* Gyro conversation from LSB -> rad/s
 * depending on the value of FS_SEL
 */
#define GYRO_FS_SEL_250dgps     0
#define GYRO_FS_SEL_500dgps     1
#define GYRO_FS_SEL_1000dgps    2
#define GYRO_FS_SEL_2000dgps    3
#define GYRO_RAW_TO_VEL(FS_SEL) ( 4.36332313*((float)(1 << FS_SEL))/32767.0 )

/* Accel conversation from LSB -> rad/s
 * depending on the value of AFS_SEL
 */
#define ACCEL_AFS_SEL_2g        0
#define ACCEL_AFS_SEL_4g        1
#define ACCEL_AFS_SEL_8g        2
#define ACCEL_AFS_SEL_16g       3
#define ACCEL_RAW_TO_REAL(AFS_SEL) ( 2.0*((float)(1 << AFS_SEL))/32767.0 )

// Structs for storing results
#define IMU_OVERFLOW_AX_OFFSET 0
#define IMU_OVERFLOW_AY_OFFSET 1
#define IMU_OVERFLOW_AZ_OFFSET 2
#define IMU_OVERFLOW_GX_OFFSET 3
#define IMU_OVERFLOW_GY_OFFSET 4
#define IMU_OVERFLOW_GZ_OFFSET 5
#define IMU_OVERFLOW_MX_OFFSET 6
#define IMU_OVERFLOW_MY_OFFSET 7
#define IMU_OVERFLOW_MZ_OFFSET 8
typedef struct tag_imu_store_struct {
    float accX;
    float accY;
    float accZ;
    float temp;
    float gyroX;
    float gyroY;
    float gyroZ;
    float mX;
    float mY;
    float mZ;
    UINT16 overflow;
} imu_store_struct;

typedef struct tag_imu_raw_store_struct {
    INT16 accX;
    INT16 accY;
    INT16 accZ;
    INT16 gyroX;
    INT16 gyroY;
    INT16 gyroZ;
} imu_raw_store_struct;

typedef struct tag_imu_calib_struct {
    float axScale;
    INT16 axBias;
    float ayScale;
    INT16 ayBias;
    float azScale;
    INT16 azBias;
    float gxScale;
    INT16 gxBias;
    float gyScale;
    INT16 gyBias;
    float gzScale;
    INT16 gzBias;
    INT16 mxBias, myBias, mzBias;
    UINT8 asax, asay, asaz; // magnetometer sensitivity adjustments
} imu_calib_struct;

// Function prototypes
// Return 0 if succesful
UINT8 init_mpu9150(void);

// Reads data
UINT8 read_imu_data(imu_store_struct*);

// Read raw data
UINT8 read_raw_imu_data(imu_raw_store_struct*);

/*
 * Gets an angle measurement using only the accelerometer given the
 * imu data in the provided struct
 */
float get_measurement(imu_store_struct* imu_data);

/*
 * Get absolute acc on imu, should be 1g for systems in rest
 */
float get_abs_acc(imu_store_struct* imu_data);

/*
 * Gets the absolute acceleration in the plane of
 * rotation
 */
float get_abs_plane_acc(imu_store_struct* imu_data);

/*
 * Get gyro control signal
 */
float get_control_signal(imu_store_struct* imu_data);

/*
 * Performs one step of a kalman filtered value of the angle.
 *
 * Must call this once every FS as defined in main_declarations.h
 * in order to integrate correctly
 */
float kalman_filtering(imu_store_struct* imu_data, UINT8* reset);

/*
 * Complementary filter
 */
float complementary_filter(imu_store_struct* imu_data);

#endif