/* 
 * File:   main_init_funcs.h
 * Author: vkozma
 *
 * Some init functions and macros used by the pic24
 *
 * Initiates the encoders and timers as well as 
 * providing the interrupt routines for the encoders
 *
 * Created on May 4, 2015
 */

#ifndef MAIN_INIT_FUNCS_H
#define	MAIN_INIT_FUNCS_H

#include "main_declarations.h"

// TODO:implement
// Use the index pin to continuously calibrate the encoder
// to combat encoder drifting.
//#define USE_INDEX_CHANGE_NOTIFICATION_INTERRUPT 


/*
 * Initialize oscillator
 */
void init_oscillator(void);

/*
 * Initializes the encoders and their respective interrupts
 * Additionally defines macros for reading the current encoder state
 */
#define GET_INT0_STATE() (PORTBbits.RB7)
#define GET_INT1_STATE() (PORTBbits.RB14)
#define GET_INDEX_STATE() (PORTAbits.RA1)
void init_encoders(void);

/*
 * Calculates and sets the best period for timer1 to achieve the
 * required sampling period. Input is sampling frequency
 */
void set_timer1_period(unsigned int Fs);

/*
 * Initiaties timers for timer interrupt and clock cycle counting
 * Additionally, defines macros for reading from the 32-bit timers
 */
#define READ_TMR23() (*((unsigned long int*) &TMR2))
#define READ_PR23() (*((unsigned long int*) &PR2))
#define ZERO_TMR23() (TMR2 = 0,TMR3 = 0)
#define SET_PR23(x) (PR2 = (x & 0xFFFF), PR3 = ((x >> 16) & 0xFFFF) )
#define READ_TMR45() (*((unsigned long int*) &TMR4))
#define READ_PR45() (*((unsigned long int*) &PR4))
#define ZERO_TMR45() (TMR4 = 0,TMR5 = 0)
#define SET_PR45(x) (PR4 = (x & 0xFFFF), PR5 = ((x >> 16) & 0xFFFF) )
void init_timers(unsigned int Fs);


/*
 * Does a microsecond delay with internal loops
 */
void delay_us(unsigned int us);

/*
 * Does a millisecond delay with internal loops
 */
void delay_ms(unsigned int ms);

/*
 * Error loops, goes into an infinite loop when an error occurs
 */

// Eeprom read error, blink with period 3 sec
void eepromErrorLoop();


#endif	/* MAIN_INIT_FUNCS_H */

