/*
 * File:   control_tuner.h
 * Author: vkozma
 *
 * Created on December 11, 2015, 2:47 PM
 */

#ifndef CONTROL_TUNER_H
#define	CONTROL_TUNER_H

#include "main_declarations.h"



// Possible controllers
#define PD_POSITION_CONTROLLER      0
#define PID_POSITION_CONTROLLER     1
#define PI_SPEED_CONTROLLER         2

// Return values for these functions
#define CONTROL_TUNING_SUCCESFUL    0
#define CONTROL_TUNING_FAILED       1

// If two poles are required they will make them a complex conjugate where
// the real part will be a factor times the provided pole and the imaginary part
// another factor times this pole. These two factors has the relationship
// w^2 = DOUBLE_POLE_REAL_FACTOR^2 + DOUBLE_POLE_IMAG_FACTOR^2
#define DOUBLE_POLE_REAL_FACTOR     0.99
#define DOUBLE_POLE_IMAG_FACTOR     0.1407

/*
 * Tune the controller given the input parameters J (inertia) and w (the closed
 * loop pole). It also needs the type of controller which are defined above.
 *
 * Stores the result in newControlParams
 *
 * Returns 0 if succesful, 1 if failed for some reason
 */
unsigned char tune_control_parameters(unsigned char type, float J, float w);

/*
 * Tunes a PD position controller given the input parameters
 */
unsigned char PD_position_control_parameters(control_params_struct* cp, motor_params_struct* mtr, float J, float w);

/*
 * Tunes a PI speed controller given the input parameters
 */
unsigned char PI_speed_control_parameters(control_params_struct* cp, motor_params_struct* mtr, float J, float w);

/*
 * Tunes a PID position controller given the input parameters
 */
unsigned char PID_position_control_parameters(control_params_struct* cp, motor_params_struct* mtr, float J, float w);

/*
 * Tunes a cascaded error feedback controller which uses the speed as input.
 * Useful for feedback in trajectory planning algorithms.
 */
#define CASCACED_CONTROLLER_CLOSED_LOOP_POLE -6.0
// k is the closed loop pole of the velocity controller that this cascaded
// controller is supposed to control
unsigned char PD_cascaded_position_controller(float k,float* P,float* D);
unsigned char PID_cascaded_position_controller(float k,float* P,float* I,float* D);

#endif	/* CONTROL_TUNER_H */

