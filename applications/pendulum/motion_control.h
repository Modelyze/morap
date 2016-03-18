/* 
 * File:   motion_control.h
 * Author: vkozma
 *
 * Contains control algorithms for the inverted
 * pendulum control.
 * 
 * Created on March 18, 2016, 1:31 PM
 */

#ifndef MOTION_CONTROL_H
#define	MOTION_CONTROL_H

#include "mpu9150.h"

// Disables all control
void disable_control();

// Does the controlling logic, starts and stops control
// when th1 and th2 is within the correct intervals
void control_logic(float th1, float th2, imu_store_struct* imu_data);

// Feedbacks loop used by control logic
float PID_feedback_loop(float th1, float th2,float* ref_th2, UINT8 reset); // PID feedback loop
float state_feedback_loop(float th1, float th2, float r, imu_store_struct* imu_data, UINT8 reset); // state feedback


#endif	/* MOTION_CONTROL_H */

