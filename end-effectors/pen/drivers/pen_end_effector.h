/* 
 * File:   pen_end_effector.h
 * Author: Viktor Kozma
 *
 * Defines parameters and macros for communicating and
 * controlling a pen end-effector
 *
 * Created on Jan 14, 2016
 */

#ifndef PEN_END_EFFECTOR_H
#define	PEN_END_EFFECTOR_H

#include "../../../api/pic32/modular_arms.h" // Path to modular_arms.h

// End effector parameters
#define END_EFFECTOR_PEN_ADDRESS    0b0011001
#define END_EFFECTOR_PEN_DEPLOY     1
#define END_EFFECTOR_PEN_RETRACT    2
#define END_EFFECTOR_PEN_SET        7

// Deploy/retract macros
#define deploy_pen() (node_write_command(END_EFFECTOR_PEN_ADDRESS,END_EFFECTOR_PEN_DEPLOY))
#define retract_pen() (node_write_command(END_EFFECTOR_PEN_ADDRESS,END_EFFECTOR_PEN_RETRACT))
#define set_pen_pos(b) (node_write_byte(END_EFFECTOR_PEN_ADDRESS,END_EFFECTOR_PEN_SET,b))

// Poke the pen address to see if a pen is on the bus
// returns 0 if success
#define poke_pen() (PokeAddress(END_EFFECTOR_PEN_ADDRESS))

#endif