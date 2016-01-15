/* 
 * File:   LMD18200_drivers.h
 * Author: vkozma
 *
 * Drivers used to control motors through the LMD18200
 * motor control chip. Includes functions and macros for
 * running the motors and reading the current.
 *
 * Designed to work with the microcontroller
 * PIC24F32KA301/302 with the following pin allocations:
 * ENABLE PIN:          RB12
 * BRAKE PIN:           RB13
 * PWM PIN:             RA6/OC1
 * THERMAL FLAG PIN:    RB15
 * CURRENT SENSE PIN:   RA0/AN0
 *
 * The PWM mode is of simple locked anti-phase type. 50 %
 * duty cycle on the PWM pin results in zero drive, 100 %
 * results in fully right and 0 % fully left. This allows
 * for an additional enable pin functionality.
 *
 * Data sheet: http://www.ti.com/lit/ds/symlink/lmd18200.pdf
 *
 * Created on May 4, 2015
 */

#ifndef LMD18200_DRIVERS_H
#define	LMD18200_DRIVERS_H

#include "main_declarations.h"
    
/*
 * Defines macros for controlling the motor
 */
#define MOTOR_ENABLE() (LATBbits.LATB12 = 1)
#define MOTOR_DISABLE() (LATBbits.LATB12 = 0)
#define MOTOR_ENABLE_SWAP() (LATBbits.LATB12 = ~(LATBbits.LATB12))
#define MOTOR_IS_ENABLED() (LATBbits.LATB12)

/*
 * If brake is on while stationary the motor will run freely,
 * if off it will be unmovable. If ON while running it won't turn,
 * if off it will turn freely.
 */
#define MOTOR_BRAKE_ON() (LATBbits.LATB13 = 1)
#define MOTOR_BRAKE_OFF() (LATBbits.LATB13 = 0)
#define MOTOR_BRAKE_IS_ON() (LATBbits.LATB13)

#define GET_THERMAL_FLAG() (PORTBbits.RB15) // If low, warning

// PWM-period in timer ticks loaded into the period register
// Needs to be uneven
#define MOTOR_PWM_PERIOD 127

// Offset the falling edge by fractions of a clock cycle using
// the DCB field, this needs to be configured if changing periods
#define DCB_OFFSET 1


// Current sensing
#define CURRENT_SENSE_RESISTOR 820.0

// The supply voltage to the motor
#define SUPPLY_VOLTAGE 24.0
// Limit voltage, is usually the same as supply voltage
#define LIMIT_VOLTAGE 20.0
// Minimum voltage, outputs below this won't get passed to the output
#define MINIMUM_VOLTAGE 0.5

// What type of PWM mode that should be used and
// whether or not the DCB bits in OC1CON2 should be
// used to increase the output resolution
#define USE_CENTER_ALIGNED_PWM
#define USE_DCB_BITS
    
/*
 * Initiates the motor pins on the board as well as the PWM timer and ADC 
 * for current sensing
 */
void init_motor(void);

/*
 * Sets the motor speed, input should be between +- the supply voltage for
 * full left and full right
 */
void set_motor_speed(float spd);

/*
 * If called with task = 0 it will perform all necessary floating point
 * calculations but not update the registers. Calling it with task = 1
 * will update the registers.
 *
 * Useful when you know you want to reduce computation time when updating
 * the motor speed when you know the speed ahead of time
 */
void set_delayed_motor_speed(float spd, unsigned char task);
#define save_motor_speed(spd) set_delayed_motor_speed(spd,0);
#define update_motor_speed() set_delayed_motor_speed(0.0,1);

/*
 * Sets the duty cycle, the input needs to be 0 <= dc <= 100
 */
void set_duty_cycle(float dc);

/*
 * Sets the OCR1 period directly, NOT implemented correctly
 */
#ifdef USE_CENTER_ALIGNED_PWM
void set_PWM_period(unsigned int rs, unsigned int r, unsigned int dcb);
#else
void set_PWM_period(unsigned int p);
#endif

/*
 * Reads the current duty cycle the motors are running at
 */
float get_duty_cycle(void);

/*
 * ADC reading functions, call ADC_sample() to start the sampling and then call
 * ADC_read to read the 12-bit sampled value.
 */
void ADC_sample(void);
unsigned int ADC_read(void);

/*
 * Read the current running through the motor
 */
float read_current(void);

/*
 * Read the voltage currently applied to the motor
 */
float read_voltage(void);

#endif	/* LMD18200_DRIVERS_H */

