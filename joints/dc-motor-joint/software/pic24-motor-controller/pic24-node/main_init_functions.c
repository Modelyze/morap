#ifndef MAIN_INIT_FUNCS_C
#define	MAIN_INIT_FUNCS_C

#include "main_declarations.h"
#include "main_init_functions.h"
#include <xc.h>

//Configuration bit settings
// To generate pragma commands in MPLABX:
// Window -> PIC memory view -> Configuration Bits -> Generate Source Code

// FBS
#pragma config BWRP = OFF               // Boot Segment Write Protect (Disabled)
#pragma config BSS = OFF                // Boot segment Protect (No boot program flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Write Protect (General segment may be written)
#pragma config GSS0 = OFF               // General Segment Code Protect (No Protection)


// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Select (Fast RC Oscillator with Postscaler and PLL Module (FRCDIV+PLL))
#pragma config SOSCSRC = DIG            // SOSC Source Type (Digital Mode for use with external source)
#pragma config LPRCSEL = HP             // LPRC Oscillator Power and Accuracy (High Power, High Accuracy Mode)
#pragma config IESO = OFF               // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-speed Start-up disabled))

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
#pragma config OSCIOFNC = OFF           // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)

/*
// FOSCSEL
#pragma config FNOSC = PRI              // Oscillator Select (Primary Oscillator (XT, HS, EC))
#pragma config SOSCSRC = DIG            // SOSC Source Type (Digital Mode for use with external source)
#pragma config LPRCSEL = HP             // LPRC Oscillator Power and Accuracy (High Power, High Accuracy Mode)
#pragma config IESO = ON                // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-speed Start-up enabled))
// FOSC
#pragma config POSCMOD = HS             // Primary Oscillator Configuration bits (HS oscillator mode selected)
#pragma config OSCIOFNC = ON            // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)
*/

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32768)
#pragma config FWPSA = PR128            // WDT Prescaler bit (WDT prescaler ratio of 1:128)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bits (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected(windowed WDT disabled))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware, SBOREN bit disabled)
#pragma config LVRCFG = OFF             //  (Low Voltage regulator is not available)
#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Use Default SCL1/SDA1 Pins For I2C1)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
#pragma config MCLRE = ON               // MCLR Pin Enable bit (RA5 input pin disabled,MCLR pin enabled)

// FICD
#pragma config ICS = PGx1               // ICD Pin Placement Select bits (EMUC/EMUD share PGC1/PGD1)

// FDS
#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

void init_oscillator(void) {

    // Internal ocsillator with PLL
    // 8 MHz x 4 PLL from FRC
    CLKDIVbits.RCDIV = 0;
    CLKDIVbits.DOZEN = 0;


    // External oscillator
//    // DOZEN disabled; DOZE 1:8; RCDIV FRC/1; ROI disabled;
//    CLKDIVbits.RCDIV = 0;
//    CLKDIVbits.DOZE = 0b011;
//    CLKDIVbits.ROI = 0;
//    CLKDIVbits.DOZEN = 0;
}


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

void delay_us(unsigned int us) {

    unsigned long int count_to_val,i;
    count_to_val = ((unsigned long int) ( (float) ((unsigned long int) us)*Fcy/1000000L)/13.0);
    for(i = 0; i < count_to_val; i++);
}

void delay_ms(unsigned int ms) {

    unsigned long int count_to_val,i;
    count_to_val = ((unsigned long int) ( (float) ((unsigned long int) ms)*Fcy/1000L)/13.0);
    for(i = 0; i < count_to_val; i++);
}

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

#endif