/* 
 * File:   modular_arms.h
 * Author: Viktor Kozma
 *
 * Communicates with motor nodes in the distributed
 * system.
 *
 * Also includes functions to write and read from the i2c bus
 *
 * All the message functions will return a status variable
 * of the transmission. This variable can take the following
 * values:
 *  0 - Success
 *  1 - NACK recieved during address transmission
 *  2 - NACK recieved during data transmission
 *  3 - Collision occured during start
 *  4 - Failed to transmit
 *  5 - I2C receiver overflow
 *
 * Created on May 19, 2015
 */

#ifndef MODULAR_ARMS_H
#define	MODULAR_ARMS_H

#include "pic32_i2c.h"


// This will make this print debug messages to the computer
//#define PRINT_DEBUG_MESSAGES

// Definitions of the motor node chips
#define MOTOR_CONTROL_CODE 0b0010
#define ID_TO_ADDRESS(id) ((MOTOR_CONTROL_CODE << 3) | id)

// I2C clock frequency used by the system
#define I2C_CLOCK_FREQ             100000

/*
* Identification codes for the I2C protocol
*/
#define I2C_DISABLE_MOTOR                   0
#define I2C_SET_POSITION_REFERENCE          1
#define I2C_SET_SPEED_REFERENCE             2
#define I2C_SET_DRIVE_VOLTAGE               3

#define I2C_SET_TRAJECTORY                  7

#define I2C_DISABLE_BRAKE                   14
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

// Motor status
#define MOTOR_STATUS_ENABLED                    1
#define MOTOR_STATUS_DISABLED                   2
#define MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED 3

// Control programming status
#define MOTOR_CONTROL_PROGRAMMING_STATUS_NOTHING    1
#define MOTOR_CONTROL_PROGRAMMING_STATUS_SUCCESS    2
#define MOTOR_CONTROL_PROGRAMMING_STATUS_FAILED     3

// Some goto parameters for tuning the controller
// Possible controller types
#define TUNING_PD_POSITION_CONTROLLER   0
#define TUNING_PID_POSITION_CONTROLLER  1
#define TUNING_PI_SPEED_CONTROLLER      2
// Inertias (check plausible inertias)
#define TUNING_LOW_INERTIA          0.05
#define TUNING_MEDIUM_INERTIA       0.2
#define TUNING_HIGH_INERTIA         0.5
#define TUNING_HUGE_INERTIA         1.0
#define TUNING_HUMONGOUS_INERTIA    1.5
// Poles
#define TUNING_POSITION_POLE_FAST   -12.0
#define TUNING_POSITION_POLE_MEDIUM -9.0
#define TUNING_POSITION_POLE_SLOW   -6.0
#define TUNING_SPEED_POLE_FAST      -60.0
#define TUNING_SPEED_POLE_MEDIUM    -40.0
#define TUNING_SPEED_POLE_SLOW      -20.0



// Control parameter struct (TODO: fix)
#define CMODE_POSITION_CONTROL 0
#define CMODE_SPEED_CONTROL 1
typedef struct tag_control_params_struct {
    unsigned char cMode;    // 0 = position control, 1 = speed control
    unsigned int Fs;        // Sampling frequency
    unsigned char nd;       // number of feedback numerator constants
    float d[4];             // Feedback numerator constants
    unsigned char nc;       // number of feedforward numerator constants
    float c[4];             // Feedforward numerator constants
    unsigned char nf;       // number of feedback/forward denominator constants
    float f[4];             // Feedback/forward denominator constants
    float I;                // Optional integrator for both feedback/forward
} control_params_struct;



#ifdef PRINT_DEBUG_MESSAGES
// externally defined output buffer for printing debug messages
extern unsigned char buf[256];
#endif

/*
 * Functions for communicating with the motor node
 */

/*
 * Initiates the I2C module on the system to communicate with the
 *
 * input:
 *  current system peripheral clock
 *
 * output:
 *  none
 *
 * example code:
 *  init_arm(SYS_FREQ/1); // Depends on you current peripheral clock
 */
void init_arm(UINT32 system_peripheral_clock);

/*
 * Reads a float from the node
 *
 * input:
 *  id of the motor
 *  id of what parameter you want to read
 *  pointer to where to store the result
 *
 * output:
 *  i2c status of the message
 *
 * example code: 
 *  // Read the angle of the motor
 *  float motor_angle;
 *  UINT8 i2c_status = node_read_float(1,I2C_READMODE_ANGLE,&motor_angle);
 *  if (i2c_status == I2C_STATUS_SUCCESFUL) {
 *      // read succesful, do something with motor_angle
 *  }
 *
 */
UINT8 node_read_float(UINT8 address, UINT8 read_id, float* output);

/*
 * Reads a single byte from the node
 *
 * input:
 *  id of the motor
 *  id of what parameter you want to read
 *  pointer to where to store the result
 *
 * output:
 *  i2c status of the message
 *
 * example code:
 *  // Read the status of the motor
 *  UINT8 i2c_status, node_status;
 *  i2c_status = node_read_float(1,I2C_READMODE_STATUS,&node_status);
 *  if (i2c_status == I2C_STATUS_SUCCESFUL) {
 *      // read succesful, do something with node_status
 *  }
 * 
 */
UINT8 node_read_byte(UINT8 address, UINT8 read_id, UINT8* output);

/*
 * Writes a single command to the motor node.
 *
 * input:
 *  id of the motor
 *  the command you want to send
 *
 * output:
 *  i2c status of the message
 *
 * example code:
 *  // Send that you want to calibrate the encoder
 *  UINT8 i2c_status = node_write_command(1,I2C_CALIBRATE_ENCODER_ZERO);
 *  if (i2c_status != I2C_STATUS_SUCCESFUL) {
 *      // Transmission failed, take proper action
 *  }
 */
UINT8 node_write_command(UINT8 address, UINT8 command);

/*
 * Writes a command and a byte of data to the motor node.
 * Not used for any command to the motors but could be used for
 * communicating with other external devices
 *
 * input:
 *  id of the motor
 *  the command you want to send
 *  the data you want to write (byte)
 *
 * output:
 *  i2c status of the message
 */
UINT8 node_write_byte(UINT8 address, UINT8 command, UINT8 write_data);


/*
 * Writes a float to the motor node
 *
 * input:
 *  id of the motor
 *  what type of data you want to write
 *  the data you want to write (float)
 *
 * output:
 *  i2c status of the message
 *
 * example code:
 *  // Send reference point to controller
 *  UINT8 i2c_status = node_write_float(1,I2C_SET_REFERENCE_POINT,1.2345);
 *  if (i2c_status != I2C_STATUS_SUCCESFUL) {
 *      transmission failed, take action
 *  }
 */
UINT8 node_write_float(UINT8 address, UINT8 write_id, float write_data);

/*
 * Programs in new control parameters into the motor
 *
 * input:
 *  id of the motor
 *  pointer to the control struct with the
 *      new control parameteres
 *
 * output:
 *  i2c status of the message
 *
 * example code:
 *  // Program in new control parameters
 *  control_params_struct controlParams =
 *  {
 *      .cMode = CMODE_POSITION_CONTROL,
 *      .Fs = 200,
 *      .nd = 2, .d = {-28.79, 31.0},
 *      .nc = 2, .c = {-28.79, 31.0},
 *      .nf = 1, .f = {-0.8869},
 *      .I = 0.0
 *  };
 *  UINT8 i2c_status = program_control_params(1,&controlParams);
 *  if (i2c_status == 0) {
 *      UINT8 prog_status;
 *      i2c_status = read_control_prog_status(1, &prog_status);
 *      if(i2c_status == I2C_STATUS_SUCCESFUL && prog_status == MOTOR_CONTROL_PROGRAMMING_STATUS_SUCCESS) {
 *          // SUCCESS
 *      }
 *  }
 */
UINT8 program_control_params(UINT8 node_id, control_params_struct* new_params);

/*
 * Tunes the the motor node with respect to the parameters provided. Eventual
 * motor parameters are stored on the motor node
 *
 * input:
 *  id of the motor
 *  type of controller to program
 *  the rotational inertia to tune around
 *  the closed loop pole
 *
 * output i2c status of the message
 *
 * example code:
 *  // Tune the motor node 1
 *  UINT8 i2c_status = tune_control_params(1,TUNING_PD_POSITION_CONTROLLER,TUNING_MEDIUM_INERTIA,TUNING_POSITION_POLE_SLOW);
 *  if(i2c_status == I2C_STATUS_SUCCESFUL) {
 *      UINT8 prog_status;
 *      i2c_status = read_control_prog_status(1, &prog_status);
 *      if(i2c_status == I2C_STATUS_SUCCESFUL && prog_status == MOTOR_CONTROL_PROGRAMMING_STATUS_SUCCESS) {
 *          // SUCCESS
 *      }
 *  }
 *
 */
UINT8 tune_control_params(UINT8 node_id, UINT8 c_type, float J, float w);

/*
 * Macros for each of the commands you can send to the motor
 */
#define get_angle(nid,out) (node_read_float(ID_TO_ADDRESS(nid),I2C_READMODE_ANGLE,out))
#define get_voltage(nid,out) (node_read_float(ID_TO_ADDRESS(nid),I2C_READMODE_VOLTAGE,out))
#define get_current(nid,out) (node_read_float(ID_TO_ADDRESS(nid),I2C_READMODE_CURRENT,out))
#define get_status(nid,out) (node_read_byte(ID_TO_ADDRESS(nid),I2C_READMODE_STATUS,out))
#define get_control_prog_status(nid,out) (node_read_byte(ID_TO_ADDRESS(nid),I2C_READMODE_CONTROL_PROG_STATUS,out))

#define calibrate_encoder_zero(nid) (node_write_command(ID_TO_ADDRESS(nid),I2C_CALIBRATE_ENCODER_ZERO))
#define set_calibration_status_unknown(nid) (node_write_command(ID_TO_ADDRESS(nid),I2C_SET_CALIBRATE_STATUS_UNKNOWN))
#define disable_motor(nid) (node_write_command(ID_TO_ADDRESS(nid),I2C_DISABLE_MOTOR))
#define enable_brake(nid) (node_write_command(ID_TO_ADDRESS(nid),I2C_ENABLE_BRAKE))
#define disable_brake(nid) (node_write_command(ID_TO_ADDRESS(nid),I2C_DISABLE_BRAKE))
#define set_default_control_params(nid) (node_write_command(ID_TO_ADDRESS(nid),I2C_USE_DEFAULT_CONTROL_PARAMS))

#define set_voltage(nid,flt) (node_write_float(ID_TO_ADDRESS(nid),I2C_SET_DRIVE_VOLTAGE,flt))
#define set_angle(nid,flt) (node_write_float(ID_TO_ADDRESS(nid),I2C_SET_POSITION_REFERENCE,flt))
#define set_angular_velocity(nid,flt) (node_write_float(ID_TO_ADDRESS(nid),I2C_SET_SPEED_REFERENCE,flt))

#define poke_motor(nid) (PokeAddress(ID_TO_ADDRESS(nid)))

#endif	/* MODULAR_ARMS_H */

