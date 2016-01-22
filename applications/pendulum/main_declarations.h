/* 
 * File:   main_declarations.h
 * Author: Viktor Kozma
 *
 * Includes general declarations and functions
 * used all over this program
 *
 *
 * Created on January 17, 2016, 4:34 PM
 */

#ifndef MAIN_DECLARATIONS_H
#define	MAIN_DECLARATIONS_H

// Lengths of each individual arms
#define L1 0.257
#define L2 0.380

// SYS FREQ
#define SYS_FREQ (80000000L)

// ONBOARD LEDS
#define INIT_LEDS() (TRISGCLR = 1 << 6, TRISFCLR = 1)
#define LED4_ON() (PORTGSET = 1 << 6)
#define LED4_OFF() (PORTGCLR = 1 << 6)
#define LED4_SWAP() (PORTGINV = 1 << 6)
#define LED5_ON() (PORTFSET = 1)
#define LED5_OFF() (PORTFCLR = 1)
#define LED5_SWAP() (PORTFINV = 1)

// SHIELD LEDS
#define INIT_SHIELD_LEDS() (TRISDCLR = (1 << 1) | (1 << 2))
#define RED_LED_ON() (PORTDSET = 1 << 1)
#define RED_LED_OFF() (PORTDCLR = 1 << 1)
#define RED_LED_SWAP() (PORTDINV = 1 << 1)
#define YELLOW_LED_ON() (PORTDSET = 1 << 2)
#define YELLOW_LED_OFF() (PORTDCLR = 1 << 2)
#define YELLOW_LED_SWAP() (PORTDINV = 1 << 2)

// Shield Buttons
#define INIT_BUTS() (TRISDSET = 1 << 8 | 1 << 9)
#define GET_BUT1() (PORTDbits.RD8)
#define GET_BUT2() (PORTDbits.RD9)

// Pulse trigger for oscilloscope trigger functions
#define INIT_PULSE_TRIGGER() (TRISECLR = 1, LATECLR = 1)
#define PULSE_TRIGGER() (LATESET = 1, LATECLR = 1)

// Conversations
#define RAD_TO_DEG(rad) (57.295779*rad)

// Functions
/*
* Initates the adc module
*/
void init_adc(void);

/*
* Reads the adc from the selected adc input
*/
#define A0 0
#define A1 1
UINT16 read_adc(UINT8 channel);

/*
* Small delay in milliseconds
*/
void delay_ms(UINT32 dly);

// Global variables
extern unsigned char buf[256];//, i2c_status;
extern volatile unsigned long int millis;

#endif	/* MAIN_DECLARATIONS_H */