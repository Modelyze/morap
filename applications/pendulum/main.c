/*
 * File:   main.c
 * Author: Viktor Kozma
 *
 * Created on January 18, 2016, 2:13 PM
 *
 * Does inverse kinematics
 *
 * Electrical connections (uC loc(uno32 pin)):
 * On board uno32 leds on PORTG6 and PORTF0
 * RED LED ON PORTD1(5)
 * YELLOW LED ON PORTD2(6)
 * BUTTON 1 on PORTD8(2)
 * BUTTON 2 on PORTD9(7)
 *
 * To initiate and exit the inverse kinematics
 * press button 2.
 *
 * To change the id's of the controlled motors
 * change the NODE1_ID and/or NODE2_ID
 * definitions below. To change the lengths of the
 * links change L1 and L2 in main_declarations.h.
 * To change toolpath change the toolpath definition
 * in ik_calc.h to something defined in toolpaths.h.
 */

#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <xc.h>
#include <math.h>
#include "main_declarations.h"
#include "../../api/pic32/modular_arms.h" //path to modular_arms.h


// Configs
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#define SYS_FREQ (80000000L)

// System clocks
#define GetSystemClock()           (SYS_FREQ)
#define GetPeripheralClock()       (SYS_FREQ/1)
#define GetInstructionClock()      (SYS_FREQ)

// Id's of the motors to be controlled
#define NODE_ID 1

// Sampling rate (Hz)
#define FS 100

// Global vars
unsigned char buf[256]; // OUTPUT BUFFER FOR UART

// function prototypes TODO: implement
void init_gyro();
//void

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

    // init adc
    init_adc();

    // Button and LEDS
    INIT_LEDS();
    LED4_OFF(); LED5_OFF();
    INIT_SHIELD_LEDS();
    RED_LED_OFF(); YELLOW_LED_OFF();
    INIT_BUTS();

    // INIT the arm
    init_arm(GetPeripheralClock());

    // says hello
    sprintf(buf,"Systems initiated\n\r");
    putsUART1(buf);

    // Enables interrupts
    INTEnableSystemMultiVectoredInt();


    while(1){
        // main loop
    }
}


// Timer 1 interrupt routine
void __ISR(_TIMER_1_VECTOR, IPL2AUTO) _Timer1Handler(void) {
    mT1ClearIntFlag();
    // timer 1 interrupt for pendulum control
}