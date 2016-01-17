/* 
 * File:   logging_routines.h
 * Author: Viktor Kozma
 *
 * Logging routines used for logging individual joint
 * 
 * All of these functions stays in their functions for the
 * duration of the logging routine.
 * 
 * Created on January 17, 2016, 4:37 PM
 */
 
#ifndef LOGGING_ROUTINES_H
#define LOGGING_ROUTINES_H

/*
* Logs the step response from a set applied voltage 
*/
void TransientLogging(UINT8 node_id, float log_voltage);

/*
* Steps through some random voltages to record maximum speed achieved at a certain
* voltage
*/
void RandomLogging(UINT8 node_id, float max_voltage);

/*
* Logs a step response from a control input signal
*/
void ControlLogging(UINT8 node_id);

#endif