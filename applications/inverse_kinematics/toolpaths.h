/*
 * File:   toolpaths.h
 * Author: Viktor Kozma
 *
 * Different toolpaths for moving placing the end-effector
 * at certain positions. 
 *
 * Created on January 18, 2016, 4:12 PM
 */

#ifndef TOOLPATHS_H
#define TOOLPATHS_H

#include "main_declarations.h"

/*
 * Toolpath element struct. Each element contains one command
 * variable command (as defined below) and up to two floats
 * that can be used to issue the commands.
 */
typedef struct tag_element {
    UINT8 type;
    float e1;
    float e2;
} element;
// Do nothing for the time defined by e1
#define ELEMENT_TYPE_DO_NOTHING             0
// Move instantly to the position x = e1, y = e2
#define ELEMENT_TYPE_MOVE_TO_POS            1
// Sets the speed that the end effector should be moving during interpolation as e1
#define ELEMENT_TYPE_SET_SPEED              2
// Interpolates positions between the last position and the one defined by x = e1, y = e2
#define ELEMENT_TYPE_INTERPOLATE_POS        3
// Directly set the angles instead of end effector position
#define ELEMENT_TYPE_SET_ANGLES             4
// Deploy/retract end effectors
#define ELEMENT_TYPE_DEPLOY_END_EFFECTOR    10
#define ELEMENT_TYPE_RETRACT_END_EFFECTOR   11
// Go to command offsets as defined by e1, useful for looping paths
#define ELEMENT_TYPE_GO_TO_ABS_COMMAND      32
#define ELEMENT_TYPE_GO_TO_REL_OFFSET       33
// Exit run
#define ELEMENT_TYPE_EXIT                   255

/*
 * If not set by a command this value will be used as a default toolpath speed
 * for the interpolate_pos command
 */
#define DEFAULT_SPEED   0.1
// Delay before next command at the end of a interpolate position command
#define INTERPOLATE_POS_END_DELAY 0.25

typedef struct toolpath_struct_tag {
    UINT8 size;
    element* path;
} toolpath_struct;

/*
 * Does a square with some crosses in the middle
 */
extern const toolpath_struct square_path;


/*
 * Repeatedly draws lines
 */
extern const toolpath_struct line_path;


#endif

