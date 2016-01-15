/* 
 * File:   pic24_i2c_slave.h
 * Author: vkozma
 *
 * Created on May 8, 2015, 12:20 PM
 */

#ifndef PIC24_I2C_SLAVE_H
#define	PIC24_I2C_SLAVE_H

#include "main_declarations.h"


#define I2C_CLOCK_FREQ             100000

// Clock stretching is needed since the i2c interrupt has low priority and
// won't be able to answer immidently some times. With clock stretch the
// slave holds the clock line low until the message is handled
#define ENABLE_I2C_CLOCK_STRETCH


/*
 * Identification codes for the I2C protocol
 */
#define I2C_DISABLE_MOTOR                   0
#define I2C_SET_POSITION_REFERENCE          1
#define I2C_SET_SPEED_REFERENCE             2
#define I2C_SET_DRIVE_VOLTAGE               3

#define I2C_SET_TRAJECTORY                  7

#define I2C_ENABLE_BRAKE                    15

#define I2C_CALIBRATE_ENCODER_ZERO          16
#define I2C_CALIBRATE_ENCODER_PROVIDED      17
#define I2C_SET_CALIBRATE_STATUS_UNKNOWN    31

#define I2C_PROGRAM_NEW_CONTROL_PARAMS      32
#define I2C_USE_DEFAULT_CONTROL_PARAMS      33
#define I2C_TUNE_CONTROL_PARAMS             34

#define I2C_READMODE_ANGLE                  128
#define I2C_READMODE_POS_REFERENCE          129
#define I2C_READMODE_VEL_REFERENCE          130
#define I2C_READMODE_STATUS                 131
#define I2C_READMODE_CONTROL_PROG_STATUS    132
#define I2C_READMODE_VOLTAGE                133
#define I2C_READMODE_CURRENT                134
#define I2C_READMODE_WHO_AM_I               136
#define I2C_READMODE_POS_CONTROL_PARAMS     138
#define I2C_READMODE_VEL_CONTROL_PARAMS     139

#define I2C_TURN_ON_LED                     252
#define I2C_TURN_OFF_LED                    253
#define I2C_SWAP_LED                        254

#ifdef	__cplusplus
extern "C" {
#endif




/*
 * Initiates the i2c module as a slave.
 * The address consists of the predefined control code in
 * addition to the provided general motor id number
 */
void init_i2c(unsigned char id);



#ifdef	__cplusplus
}
#endif

#endif	/* PIC24_I2C_SLAVE_H */

