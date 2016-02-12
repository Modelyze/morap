#ifndef _MPU9150_C
#define	_MPU9150_C

#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <xc.h>
#include <math.h>
#include "main_declarations.h"
#include "mpu9150.h"
#include "../../api/pic32/pic32_i2c.h"

// Calibration data for a specific imu, each device has to be calibrated on its
// own
/*
imu_calib_struct calib = {.axBias = -291, .ayBias = 53, .azBias = -327, \
                          .gxBias = 365, .gyBias = 90, .gzBias = -196, \
                          .mxBias = 0, .myBias = 0, .mzBias = 0,
                          .asax = 128, .asay = 128, .asaz = 128};
*/
imu_calib_struct calib = {.axBias = -74, .ayBias = 80, .azBias = -172, \
                          .gxBias = -13, .gyBias = -88, .gzBias = -58, \
                          .mxBias = 0, .myBias = 0, .mzBias = 0,
                          .asax = 128, .asay = 128, .asaz = 128};
// The accelerometers which point in the directions
const float a_axial[3] = {0,-1,0};
const float a_radial[3] = {1,0,0};
const float g_rate[3] = {0,0,1};

// Function prototypes
UINT8 init_mpu9150(void) {
    UINT8 data[2], ret_val;
    // Reset device and wait a bit
    data[0] = MPU9150_PWR_MGMT_1;
    data[1] = 0x80;
    ret_val = TransmitData(MPU9150_I2C_ADDRESS,data,2); 
    if (ret_val == I2C_STATUS_SUCCESFUL) delay_ms(10);
    else return ret_val;
    // Enable bypass mode to directly access magnetometer
    data[0] = MPU9150_INT_PIN_CFG;
    data[1] = 0b00000010;
    ret_val = TransmitData(MPU9150_I2C_ADDRESS,data,2); delay_ms(1);
    // Disable sleep mode
    data[0] = MPU9150_PWR_MGMT_1;
    data[1] = 0;
    ret_val |= TransmitData(MPU9150_I2C_ADDRESS,data,2); delay_ms(1);
    // Sets gyro output to +- 500dg/s
    UINT8 FS_SEL = GYRO_FS_SEL_500dgps;
    data[0] = MPU9150_GYRO_CONFIG;
    data[1] = (FS_SEL << 3);
    ret_val |= TransmitData(MPU9150_I2C_ADDRESS,data,2); delay_ms(1);
    // Sets accel output to +- 4g
    UINT8 AFS_SEL = ACCEL_AFS_SEL_4g;
    data[0] = MPU9150_ACCEL_CONFIG;
    data[1] = (AFS_SEL << 3);
    ret_val |= TransmitData(MPU9150_I2C_ADDRESS,data,2); delay_ms(1);
    // Sets DLPF to a around 100 Hz, this ensures sampling rate of imu to be 1 kHz
    data[0] = MPU9150_CONFIG;
    data[1] = (3 << 0);
    ret_val |= TransmitData(MPU9150_I2C_ADDRESS,data,2); delay_ms(1);

    // Load device calibration values
    calib.axScale = ACCEL_RAW_TO_REAL(AFS_SEL);
    calib.ayScale = ACCEL_RAW_TO_REAL(AFS_SEL);
    calib.azScale = ACCEL_RAW_TO_REAL(AFS_SEL);
    calib.gxScale = GYRO_RAW_TO_VEL(FS_SEL);
    calib.gyScale = GYRO_RAW_TO_VEL(FS_SEL);
    calib.gzScale = GYRO_RAW_TO_VEL(FS_SEL);

    sprintf(buf,"axScale = %0.6f, gxScale = %0.6f\n\r",calib.axScale,calib.gxScale);
    putsUART1(buf);

    // Read magnetometer sensitivity adjustment values
    data[0] = AK8975C_ASAX;
    ret_val |= TransmitData(AK8975C_I2C_ADDRESS,data,1); delay_ms(1);
    ret_val |= ReadData(AK8975C_I2C_ADDRESS,&calib.asax,1); delay_ms(1);
    data[0] = AK8975C_ASAY;
    ret_val |= TransmitData(AK8975C_I2C_ADDRESS,data,1); delay_ms(1);
    ret_val |= ReadData(AK8975C_I2C_ADDRESS,&calib.asay,1); delay_ms(1);
    data[0] = AK8975C_ASAZ;
    ret_val |= TransmitData(AK8975C_I2C_ADDRESS,data,1); delay_ms(1);
    ret_val |= ReadData(AK8975C_I2C_ADDRESS,&calib.asaz,1); delay_ms(1);

    return ret_val;
}

// Read and convert data from imu and store in a struct
#define ACCX_OFFSET     0
#define ACCY_OFFSET     2
#define ACCZ_OFFSET     4
#define TEMP_OFFSET     6
#define GYROX_OFFSET    8
#define GYROY_OFFSET    10
#define GYROZ_OFFSET    12
UINT8 read_imu_data(imu_store_struct* imu_data) {
    UINT8 data[14], ret_val;
    // Read raw data
    data[0] = MPU9150_ACCEL_XOUT_H;
    ret_val = TransmitData(MPU9150_I2C_ADDRESS,data,1);
    if (ret_val == I2C_STATUS_SUCCESFUL) {
        ret_val = ReadData(MPU9150_I2C_ADDRESS,data,14);
    } else {
        return ret_val;
    }
    if (ret_val != I2C_STATUS_SUCCESFUL) {
        return ret_val;
    }

    // Convert data to useful stuff
    INT16 raw_accX = (INT16) (data[ACCX_OFFSET] << 8) | data[ACCX_OFFSET+1];
    INT16 raw_accY = (INT16) (data[ACCY_OFFSET] << 8) | data[ACCY_OFFSET+1];
    INT16 raw_accZ = (INT16) (data[ACCZ_OFFSET] << 8) | data[ACCZ_OFFSET+1];
    INT16 raw_temp = (INT16) (data[TEMP_OFFSET] << 8) | data[TEMP_OFFSET+1];
    INT16 raw_gyroX = (INT16) (data[GYROX_OFFSET] << 8) | data[GYROX_OFFSET+1];
    INT16 raw_gyroY = (INT16) (data[GYROY_OFFSET] << 8) | data[GYROY_OFFSET+1];
    INT16 raw_gyroZ = (INT16) (data[GYROZ_OFFSET] << 8) | data[GYROZ_OFFSET+1];
    imu_data->accX = calib.axScale*((float) raw_accX + (float) calib.axBias);
    imu_data->accY = calib.ayScale*((float) raw_accY + (float) calib.ayBias);
    imu_data->accZ = calib.azScale*((float) raw_accZ + (float) calib.azBias);
    imu_data->temp = ((float) raw_temp)/340.0 + 35.0;
    imu_data->gyroX = calib.gxScale*( ((float) raw_gyroX) + ((float) calib.gxBias) );
    imu_data->gyroY = calib.gyScale*( ((float) raw_gyroY) + ((float) calib.gyBias) );
    imu_data->gyroZ = calib.gzScale*( ((float) raw_gyroZ) + ((float) calib.gzBias) );

    return ret_val;
}
UINT8 read_raw_imu_data(imu_raw_store_struct* imu_raw_data) {
    UINT8 data[14], ret_val;
    // Read raw data
    data[0] = MPU9150_ACCEL_XOUT_H;
    ret_val = TransmitData(MPU9150_I2C_ADDRESS,data,1);
    if (ret_val == I2C_STATUS_SUCCESFUL) {
        ret_val = ReadData(MPU9150_I2C_ADDRESS,data,14);
    } else {
        return ret_val;
    }
    if (ret_val != I2C_STATUS_SUCCESFUL) {
        return ret_val;
    }

    // Convert data to useful stuff
    imu_raw_data->accX = (INT16) (data[ACCX_OFFSET] << 8) | data[ACCX_OFFSET+1];
    imu_raw_data->accY = (INT16) (data[ACCY_OFFSET] << 8) | data[ACCY_OFFSET+1];
    imu_raw_data->accZ = (INT16) (data[ACCZ_OFFSET] << 8) | data[ACCZ_OFFSET+1];
    imu_raw_data->gyroX = (INT16) (data[GYROX_OFFSET] << 8) | data[GYROX_OFFSET+1];
    imu_raw_data->gyroY = (INT16) (data[GYROY_OFFSET] << 8) | data[GYROY_OFFSET+1];
    imu_raw_data->gyroZ = (INT16) (data[GYROZ_OFFSET] << 8) | data[GYROZ_OFFSET+1];
    
    return ret_val;
}


float get_abs_acc(imu_store_struct* imu_data) {
    return sqrtf(powf(imu_data->accX,2) + powf(imu_data->accY,2) + powf(imu_data->accZ,2));
}

float get_abs_plane_acc(imu_store_struct* imu_data) {
    return sqrtf(powf(a_radial[0]*imu_data->accX,2) + powf(a_axial[0]*imu_data->accX,2) + \
                 powf(a_radial[1]*imu_data->accY,2) + powf(a_axial[1]*imu_data->accY,2) + \
                 powf(a_radial[2]*imu_data->accZ,2) + powf(a_axial[2]*imu_data->accZ,2) );
}

float get_measurement(imu_store_struct* imu_data) {
    float a_rad = a_radial[0]*imu_data->accX + a_radial[1]*imu_data->accY + a_radial[2]*imu_data->accZ;
    float a_ax = a_axial[0]*imu_data->accX + a_axial[1]*imu_data->accY + a_axial[2]*imu_data->accZ;
    return atan2f(a_ax,a_rad);
}

float get_control_signal(imu_store_struct* imu_data) {
    return (g_rate[0]*imu_data->gyroX + g_rate[1]*imu_data->gyroY + g_rate[2]*imu_data->gyroZ);
}

float bind_angle(float input) {
    // binds an angle between -PI:PI degrees
    if (input < -M_PI) { // -190 deg = 170 deg
        return 2*M_PI + input;
    } else if (input > M_PI) { // 190 deg = -170 deg
        return -2*M_PI + input;
    }
    return input;
}

// Performs one step of kalman filtered position
const float Qth = 0.0002, Qbias = 0.002, Rth = 0.02; // Noise covariances
const float dT = 1/((float) FS); // Sampling time
float kalman_filtering(imu_store_struct* imu_data, UINT8* reset) {
    // Performs one step of the kalman filter and returns the
    // current estimated angle
    // Similar implementation:
    // http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
#define L_init_P 2
    static float P[4] = {L_init_P,0,0,L_init_P};
    static float x[2] = {0,0};
    float xb[2], Pb[4], K[2], Sinv, z, y;
    if(*reset) {
        P[0] = L_init_P;
        P[1] = 0;
        P[2] = 0;
        P[3] = L_init_P;
        x[0] = get_measurement(imu_data);
        x[1] = 0;
        *reset = 0;
    }

    // Predict
    xb[0] = bind_angle( x[0] + (get_control_signal(imu_data) - x[1])*dT );
    xb[1] = x[1];
    // Predict error matrix
    Pb[0] = P[0] + dT*(dT*P[3] - P[1] - P[2]) + Qth;
    Pb[1] = P[1] - dT*P[3];
    Pb[2] = P[2] - dT*P[3];
    Pb[3] = P[3] + Qbias;

    // Update
    if(fabsf(get_abs_plane_acc(imu_data) - 1.0) < 0.05 ) {
        // Calculate Kalman Gain
        Sinv = 1/(Pb[0] + Rth);
        K[0] = Pb[0]*Sinv;
        K[1] = Pb[2]*Sinv;
        // Update state with measurement
        z = get_measurement(imu_data);
        y = bind_angle(z - xb[0]);
        x[0] = bind_angle(xb[0] + K[0]*y);
        x[1] = xb[1] + K[1]*y;
        // Update error matrix
        P[0] = Pb[0]*(1.0-K[0]);
        P[1] = Pb[1]*(1.0-K[0]);
        P[2] = -K[1]*Pb[0] + Pb[2];
        P[3] = -K[1]*Pb[1] + Pb[3];
    } else {
        x[0] = xb[0];
        x[1] = xb[1];
        P[0] = Pb[0];
        P[1] = Pb[1];
        P[2] = Pb[2];
        P[3] = Pb[3];
    }

    return x[0];
}

#endif