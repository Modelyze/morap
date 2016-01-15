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
 * Initializes the encoders and their respective interrupts
 * Additionally defines macros for reading the current encoder state
 */
#define GET_INT0_STATE() (PORTBbits.RB7)
#define GET_INT1_STATE() (PORTBbits.RB14)
#define GET_INDEX_STATE() (PORTAbits.RA1)
void init_encoders(void) {
    // INT0 (RB7) + INT1 (RB14)
    TRISBbits.TRISB7 = 1;   // INPUT
    TRISBbits.TRISB14 = 1;
    ANSBbits.ANSB14 = 0;    // Digital input

    // INT0 - Encoder channel B
    INTCON2bits.INT0EP = GET_INT0_STATE(); // 1 - falling edge, 0 - rising edge
    IPC0bits.INT0IP = PRIORITY_ENCODER; // Interrupt priority
    IFS0bits.INT0IF = 0; // Clear eventual flag
    IEC0bits.INT0IE = 1; // Enable interrupt

    // INT1 - Encoder channel A
    INTCON2bits.INT1EP = GET_INT1_STATE(); // 1 - falling edge, 0 - rising edge
    IPC5bits.INT1IP = PRIORITY_ENCODER; // Interrupt priority
    IFS1bits.INT1IF = 0; // Clear flag
    IEC1bits.INT1IE = 1; // Enable interrupt

    // Index pin
    ANSAbits.ANSA1 = 0;
    TRISAbits.TRISA1 = 1;
#ifdef USE_INDEX_CHANGE_NOTIFICATION_INTERRUPT
    // Change notification interrupt on index pin
    CNEN1bits.CN3IE = 1; // = A1
    IFS1bits.CNIF = 0; // clear eventual flag
    IEC1bits.CNIE = 1;  // Enable change notificatio interrupt
#endif
}

/*
 * Calculates and sets the best period for timer1 to achieve the
 * required sampling period. Input is sampling frequency
 */
void set_timer1_period(unsigned int Fs) {
    unsigned long int period;
    unsigned int prescaler, prescalers[4] = {1,8,64,256};
    // Prescaler: 0 = 1:1, 1 = 1:8, 2 = 1:64, 3 = 1:256
    // Period calc: Ts = PR1 / ( Fcy / Prescaler )

    for (prescaler = 0; prescaler < 4; prescaler++) {
        period = (1.0/((float)Fs))*(((float)Fcy)/((float)prescalers[prescaler]));
        if (period < 0xFFFF) {
            T1CONbits.TCKPS = prescaler;
            PR1 = (unsigned int) period;
            return;
        }
    }
    // To long sample time, can't reach (1.05 sec max). Input maximum possible
    T1CONbits.TCKPS = 3;
    PR1 = 0xFFFF;
}

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
void init_timers(unsigned int Fs) {
    // Timer 1 - timer interrupt for sampling
    T1CONbits.TON = 1;
    //T1CONbits.TCKPS = 2; // 0 = 1:1, 1 = 1:8, 2 = 1:64, 3 = 1:256
    //PR1 = 1250; // Period: Ts = PR1 / ( Fcy / Prescaler )
    set_timer1_period(Fs);
    TMR1 = 0;

    // Timer 1 interrupt
    IPC0bits.T1IP = PRIORITY_TIMER; // Priority
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1; // if = 0 it's DISABLED FOR TESTING! Enable for full functionality

    // Timer45 used for clock cycle counting
    T4CONbits.T32 = 1;      // 32-bit mode
    T4CONbits.TCKPS = 0;    // 1:1 prescaler
    T4CONbits.TCS = 0;      // Interntal clock Fosc/2
    SET_PR45(0xBBBBAAAA);
    ZERO_TMR45();
    T4CONbits.TON = 1;      // Turn it on
}


/*
 * Does a microsecond delay with internal loops
 */
void delay_us(unsigned int us) {

    unsigned long int count_to_val,i;
    count_to_val = ((unsigned long int) ( (float) ((unsigned long int) us)*Fcy/1000000L)/13.0);
    for(i = 0; i < count_to_val; i++);
}

/*
 * Does a millisecond delay with internal loops
 */
void delay_ms(unsigned int ms) {

    unsigned long int count_to_val,i;
    count_to_val = ((unsigned long int) ( (float) ((unsigned long int) ms)*Fcy/1000L)/13.0);
    for(i = 0; i < count_to_val; i++);
}

/*
 * Error loops, goes into an infinite loop when an error occurs
 */

// Eeprom read error, blink with period 3 sec
void eepromErrorLoop() {
    while(1){ // No data present, stuck here
        LED2_SWAP();
        int i;
        for (i = 0; i < 100; i++)
            delay_us(15000); // 1.5 sec
    }
}

// The encoder value that the external interrupts will write to
volatile long int encoder_value = 0;

// Encoder interrupts

// Encoder channel B
void _ISR_VERY_FAST _INT0Interrupt(void) {
#ifdef LED1_MODE_ENCODER
    LED1_ON();
#endif

    if( GET_INT0_STATE() == GET_INT1_STATE() ) {
        encoder_value += 1;
    } else {
        encoder_value -= 1;
    }
    
    // Switch edge
    INTCON2bits.INT0EP = GET_INT0_STATE(); // 1 - falling edge, 0 - rising edge
    IFS0bits.INT0IF = 0;   // Clear the flag
#ifdef LED1_MODE_ENCODER
    LED1_OFF();
#endif
}

// Encoder channel A
void _ISR_VERY_FAST _INT1Interrupt(void) {
#ifdef LED1_MODE_ENCODER
    LED1_ON();
#endif
    if(GET_INT0_STATE() == GET_INT1_STATE() ) {
        encoder_value -= 1;
    } else {
        encoder_value += 1;
    }

    // Switch edge
    INTCON2bits.INT1EP = GET_INT1_STATE(); // 1 - falling edge, 0 - rising edge
    IFS1bits.INT1IF = 0; // Clear flag
#ifdef LED1_MODE_ENCODER
    LED1_OFF();
#endif
}

#ifdef USE_INDEX_CHANGE_NOTIFICATION_INTERRUPT
// Index pin change notification interrupt
void _ISR_VERY_FAST _CNInterrupt(void) {
#ifdef LED1_MODE_INDEX
    if (GET_INDEX_STATE() == 1)
        LED1_SWAP();
#endif
    IFS1bits.CNIF = 0; //clear flag
}
#endif

#endif	/* MAIN_INIT_FUNCS_H */

