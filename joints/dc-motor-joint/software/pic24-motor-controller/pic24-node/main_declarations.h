/* 
 * File:   main_declarations.h
 * Author: vkozma
 *
 * Provides declarations of macros and structs
 * used throughout all the different source files.
 *
 * Created on May 6, 2015
 */

#ifndef MAIN_DECLARATIONS_H
#define	MAIN_DECLARATIONS_H


// Oscillator and clock
#define Fosc (32000000L)
#define Fcy (Fosc/2)

// Default sampling rate when the motor is disabled
#define DEFAULT_FS 100

// Priorities for the different tasks, 0 - 7 with 7 being the highest
#define PRIORITY_ENCODER    7   // Main encoder
#define PRIORITY_INDEX      6   // Encoder index pin change interrupt
#define PRIORITY_TIMER      3   // Main timer event
#define PRIORITY_COMM       4   // I2C comm event

// ON BOARD LEDS
#ifdef __PIC24F32KA302__
#define INIT_LEDS() (TRISBbits.TRISB2 = 0, TRISBbits.TRISB4 = 0, TRISBbits.TRISB5 = 0)
#define LED3_ON() (LATBbits.LATB5 = 1)
#define LED3_OFF() (LATBbits.LATB5 = 0)
#define LED3_SWAP() (LATBbits.LATB5 = ~(LATBbits.LATB5))
#else
#define INIT_LEDS() (TRISBbits.TRISB2 = 0, TRISBbits.TRISB4 = 0)
#endif
#define LED1_ON() (LATBbits.LATB2 = 1)
#define LED1_OFF() (LATBbits.LATB2 = 0)
#define LED1_SWAP() (LATBbits.LATB2 = ~(LATBbits.LATB2))
#define LED2_ON() (LATBbits.LATB4 = 1)
#define LED2_OFF() (LATBbits.LATB4 = 0)
#define LED2_SWAP() (LATBbits.LATB4 = ~(LATBbits.LATB4))

// MODE FOR GREEN LED1 (ONLY HAVE ONE OF THESE DEFINED!!!)
#define LED1_MODE_I2C   // Flashes for a small duration when a I2C interrupt occurs
//#define LED1_MODE_ENCODER   // flashes the led1 each timer the encoder triggers
//#define LED1_MODE_INDEX     // Flashes the led each time the index pin goes high

// Flash time for LED1 in I2C mode
#define LED1_MODE_I2C_FLASH_TIME 0.05

// Encoder, how to go from encoder ticks to the output shaft angle (rad)
#define ENCODER_TO_RAD(x) (2*3.14159265/(x*4))

// I2C constants
#define I2C_CONTROL_CODE 0b0010

// INTERRUPTS
#define _ISR_NO_PSV __attribute__((interrupt, no_auto_psv))
#define _ISR_VERY_FAST __attribute__((interrupt, no_auto_psv, shadow))

// Control parameter struct
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

// Motor parameter struct for storing motor data
typedef struct tag_motor_params_struct {
    float R,L,K,n,c,eta,imax,ppr;
} motor_params_struct;

// Status struct for the system
typedef struct tag_system_status_struct {
    unsigned char enc_calib; // == 1 when position isn't known and encoder calibration is needed, 0 otherwise
    unsigned char control_prog_status;  // 1 = nothing, 2 = success, 3 = failed
    unsigned char new_control_available;    // 0 = nothing new, 1 = available, 2 = default
    unsigned char control_mode;         // 0 = pos-feedback, 1= vel-feedback, 2 = voltage control, 3 - disabled
} system_status_struct;

#define ENCODER_CALIBRATION_NEEDED 1
#define ENCODER_IS_CALIBRATED      0

#define CONTROL_PROG_STATUS_NOTHING 1
#define CONTROL_PROG_STATUS_SUCCESS 2
#define CONTROL_PROG_STATUS_FAILED  3

#define NO_NEW_CONTROLLER_AVAILABLE   0
#define NEW_CONTROLLER_AVAILABLE      1
#define NEW_CONTROLLER_DEFAULT        2

#define CONTROL_MODE_DISABLED           0
#define CONTROL_MODE_POSITION_FEEDBACK  1
#define CONTROL_MODE_SPEED_FEEDBACK     2
#define CONTROL_MODE_VOLTAGE            3
#define CONTROL_MODE_DIRECT_VOLTAGE     4
#define CONTROL_MODE_CURRENT            5
#define CONTROL_MODE_TRAJECTORY         9
#define CONTROL_MODE_NEVER              100

// Global variables
extern volatile long int encoder_value;
extern motor_params_struct motorData;
extern control_params_struct posControlParams, velControlParams, newControlParams;
extern system_status_struct systemStatus;
extern volatile float pos_ref, vel_ref, drive_voltage, amp_ref;
#ifdef LED1_MODE_I2C
extern volatile int led1_timer_count;
#endif


#endif	/* MAIN_DECLARATIONS_H */

