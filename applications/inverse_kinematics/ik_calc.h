/* 
 * File:   ik_calc.h
 * Author: Viktor Kozma
 *
 * Functions for calculating the inverse kinematics of a SCARA type arm
 *
 * Also contains routines for GCODE like definitions of path following
 * where a path can be defined using a defined interface.
 * 
 * Created on January 18, 2016, 4:13 PM
 */

#ifndef IK_CALC_H
#define	IK_CALC_H


#include "main_declarations.h"
#include "toolpaths.h"

/*
 * Calculates the inverse kinematics for a scara type arm
 * Returns 0 if succesfully capable of reaching the target, > 0 otherwise
 */
UINT8 ik_calc(float x, float y, float* a1, float* a2);

/*
 * Reads a stored tool path and returns the angles necessary to
 * arrive at that position.
 */

#define toolpath line_path // Which path to follow
UINT8 ik_path(float time, UINT8* command_type, float* a1, float* a2);


/*
 * Automatically generate a circle tool path
 */
#define CIRCLE_MID_X            (0.75*(L1+L2))
#define CIRCLE_MID_Y            0.0
#define CIRCLE_R                (0.1*(L1+L2))
#define CIRCLE_TRAVERSE_SPEED   0.01 // m/s
#define CIRCLE_ROTATIONAL_SPEED (CIRCLE_TRAVERSE_SPEED/CIRCLE_R)
#define DELAY_START_TO_CIRCLE   1.5
#define DELAY_START_TO_DEPLOY   1.2 // < DELAY_START_TO_CIRCLE
UINT8 ik_circle(float time, UINT8* command_type, float* a1, float* a2);

#endif	/* IK_CALC_H */

