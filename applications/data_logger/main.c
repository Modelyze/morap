/* 
 * File:   main.c
 * Author: Viktor Kozma
 *
 * Created on January 17, 2016, 4:23 PM
 *
 * Generates data loggings for use when extracting
 * physical modeling parameters
 * 
 * Electrical connections (uC loc(uno32 pin)):
 * On board uno32 leds on PORTG6 and PORTF0
 * RED LED ON PORTD1(5)
 * YELLOW LED ON PORTD2(6)
 * BUTTON 1 on PORTD8(2)
 * BUTTON 2 on PORTD9(7)
 * POTENTIOMETER 1 on AN2(A0)
 * POTENTIOMETER 2 on AN4(A1)
 * 
 * Toggle between different logging modes using
 * button 1 and engage selected logging mode with 
 * button 2. For transient logging select voltage
 * level with potentiometer 1.
 * 
 * To change the id of the target motor change
 * the NODE_ID in main_declarations.h
 */

#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <xc.h>
#include "main_declarations.h"
#include "../../api/pic32/modular_arms.h" //path to modular_arms.h
#include "logging_routines.h"

// Configs
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

// I2C functions!
#define GetSystemClock()           (SYS_FREQ)
#define GetPeripheralClock()       (SYS_FREQ/1)
#define GetInstructionClock()      (SYS_FREQ)


// Function prototypes
void print_current_mode(UINT8);

// global vars
UINT8 buf[256]; // OUTPUT BUFFER FOR UART
volatile UINT32 millis = 0; // Increments every 1/1000 sec

int main(void) {
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states and
    // enable prefetch cache but will not change the PBDIV. The PBDIV value
    // is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // SETS UP UART, calculating the baud rates are done as follows:
    // BAUD = FPG / (16* (U1BRG + 1))  ==>
    // U1BRG = FPG / (16*BAUD) - 1
    // Where FPG = F_CPU / FPBDIV ( 80 MHz )
    // So 129 for 38400, 42 for 115200
    // found on computer at /dev/ttyUSB0
    OpenUART1(UART_EN,UART_TX_ENABLE | UART_RX_ENABLE,42);

    // SETUP TIMER 1 INTERUPPT!
    // F_tick = (80e6 / Prescaler) / period
    //Open Timer1, 64 Prescaler -> Fclock = 1250000 Hz. With period = 1250 -> tick = 1000 Hz
    OpenTimer1(T1_ON | T1_PS_1_64 | T1_SOURCE_INT, 1250);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

    // Sets up internal 32-bit timer for counting clockcycles
    OpenTimer45(T45_ON | T45_PS_1_1, 0xFFFFFFFF);

    // Sets up ADC
    init_adc();    

    // Button and LEDS
    INIT_LEDS();
    INIT_SHIELD_LEDS();
    RED_LED_OFF(); YELLOW_LED_OFF();
    INIT_PULSE_TRIGGER();
    INIT_BUTS();
 
    //Initiates robotic arm
    init_arm(GetPeripheralClock());

    // SAYS HELLO
    sprintf(buf,"Systems initiated, id of working node: %d ...\n\r", NODE_ID);
    putsUART1(buf);

    // Enables interrupts
    INTEnableSystemMultiVectoredInt();
	
#define LOG_MODE_TRANSIENT	0 
#define LOG_MODE_RANDOM		1 
#define LOG_MODE_CONTROL	2
const UINT8 modes[] = { LOG_MODE_TRANSIENT, LOG_MODE_RANDOM, LOG_MODE_CONTROL };
#define N_MODES (sizeof(modes)/sizeof(UINT8))
#define GET_MODE(i) modes[i]
	
    UINT8 but1_pressed = 0, but2_pressed = 0, mode = 0;
    UINT32 prev_millis_but1 = 0, prev_millis_but2 = 0, prev_millis_swap = 0, prev_millis_show = 0;
    float send_value, old_send_value;
#define BUT_DEBOUNCE 10 // debouncing on buttons in milliseconds
#define SHOW_TS 100 // sampling time of showing selected voltage
    while(1){
        if (GET_BUT1() && !but1_pressed && (millis - prev_millis_but1 > BUT_DEBOUNCE)) {
            // Do stuff
            RED_LED_SWAP();
            if (millis - prev_millis_swap < 4000) {
                    mode++; if (mode >= N_MODES) mode = 0; // Chose next mode
            }
            prev_millis_swap = millis;
            print_current_mode(GET_MODE(mode));

            // Resets some other routines to avoid spam
            old_send_value = -10000.0;
            prev_millis_show = millis+1000;

            // Button management
            but1_pressed = 1;
            prev_millis_but1 = millis;
        } else if (!GET_BUT1() && but1_pressed && (millis - prev_millis_but1 > BUT_DEBOUNCE)) {
            but1_pressed = 0;
            prev_millis_but1 = millis;
        }

        // Engage mode:
        if (GET_BUT2() && !but2_pressed && (millis - prev_millis_but2 > BUT_DEBOUNCE)) {
            YELLOW_LED_ON();
            switch(GET_MODE(mode)) {
                case LOG_MODE_TRANSIENT:
                    TransientLogging(NODE_ID, send_value);
                    break;
                case LOG_MODE_RANDOM:
                    RandomLogging(NODE_ID,20.0);
                    break;
                case LOG_MODE_CONTROL:
                    ControlLogging(NODE_ID);
                    break;
            }
            YELLOW_LED_OFF();

            // Button management
            but2_pressed = 1;
            prev_millis_but2 = millis;
        } else if (!GET_BUT2() && but2_pressed && (millis - prev_millis_but2 > BUT_DEBOUNCE)) {
            but2_pressed = 0;
            prev_millis_but2 = millis;
        }

        if (millis > (SHOW_TS + prev_millis_show)) {
            if (GET_MODE(mode) == LOG_MODE_TRANSIENT) {
                send_value = 0.5*((float) ((unsigned int) (((float)(1023 - read_adc(0)))*48/1023)));
                if (send_value != old_send_value){
                    sprintf(buf,"Log voltage = %0.1f\n\r",send_value);
                    putsUART1(buf);
                    old_send_value = send_value;
                }
            }
            prev_millis_show = millis;
        }


    } // of while(1)
    return (EXIT_SUCCESS);
} // of main()

// Timer 1 interrupt routine, 1000 Hz
void __ISR(_TIMER_1_VECTOR, IPL2AUTO) _Timer1Handler(void) {
    mT1ClearIntFlag();
    // Increment global time counter
    millis++;

    // Blink led4 for visual feedback
    static UINT16 cnt = 0;
    cnt++;
    if(cnt >= 2000) {
        LED4_SWAP(); // every 2 sec
        cnt = 0;
    }
}

// Prints current mode to terminal
void print_current_mode(UINT8 mode) {
    switch(mode) {
        case LOG_MODE_TRANSIENT:
            putsUART1("- Transient Mode Selected\n\r");
            break;
        case LOG_MODE_RANDOM:
            putsUART1("- Random Voltages Mode Selected\n\r");
            break;
        case LOG_MODE_CONTROL:
            putsUART1("- Control Mode Selected\n\r");
            break;
    }
}
