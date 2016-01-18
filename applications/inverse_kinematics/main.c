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
#include "ik_calc.h"
#include "main_declarations.h"
#include "../../api/pic32/modular_arms.h" //path to modular_arms.h
#include "../../end-effectors/pen/drivers/pen_end_effector.h" // pen end effector

// Configs
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#define SYS_FREQ (80000000L)

// System clocks
#define GetSystemClock()           (SYS_FREQ)
#define GetPeripheralClock()       (SYS_FREQ/1)
#define GetInstructionClock()      (SYS_FREQ)

// Id's of the motors to be controlled
#define NODE1_ID 1
#define NODE2_ID 2

// Global vars
unsigned char buf[256]; // OUTPUT BUFFER FOR UART
volatile UINT32 millis = 0; // Increments every 1/1000 sec

// prototype
void double_joint_ik_control(UINT8*,UINT8,UINT8);

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

    // SAYS HELLO
    sprintf(buf,"toolpath length: %d\n\r",toolpath.size);
    putsUART1(buf);

    // INIT the arm
    init_arm(GetPeripheralClock());

    // Enables interrupts
    INTEnableSystemMultiVectoredInt();

    UINT8 mode_engaged = 0; // 0 = off, 1 = init, 2 = run, 3 = shut down
    UINT8 but1_pressed = 0, but2_pressed = 0;
#define BUT_DEBOUNCE 10 // ms
#define SAMPLE_DELAY 20 // ms
    UINT32 prev_millis_but1 = 0, prev_millis_but2 = 0, prev_millis = 0; // Timing
    while(1){

        if (GET_BUT1() && !but1_pressed && (millis - prev_millis_but1 > BUT_DEBOUNCE)) {
            // Do some random stuff here
            /* implement stuff if necessary*/

            prev_millis = millis; // Resets sampling to reduce spam
            // Button management
            but1_pressed = 1;
            prev_millis_but1 = millis;
        } else if (!GET_BUT1() && but1_pressed && (millis - prev_millis_but1 > BUT_DEBOUNCE)) {
            but1_pressed = 0;
            prev_millis_but1 = millis;
        }


        // Engage mode by presseing button 2
        if (GET_BUT2() && !but2_pressed && (millis - prev_millis_but2 > BUT_DEBOUNCE)) {
            if (mode_engaged) {
                mode_engaged = 3; // If engaged, tell the sequence to shut down
            } else {
                putsUART1("ENGAGED\n\r");
                YELLOW_LED_ON();
                mode_engaged = 1; // Engage sequence
            }

            prev_millis = millis; // Resets sampling to reduce spam
            // Button management
            but2_pressed = 1;
            prev_millis_but2 = millis;
        } else if (!GET_BUT2() && but2_pressed && (millis - prev_millis_but2 > BUT_DEBOUNCE)) {
            but2_pressed = 0;
            prev_millis_but2 = millis;
        }


        if ((mode_engaged > 0) && (millis - prev_millis >= SAMPLE_DELAY)) {
            prev_millis = millis;
            double_joint_ik_control(&mode_engaged,NODE1_ID, NODE2_ID);
            if (mode_engaged == 0) {
                putsUART1("DISENGAGED!\n\r");
                YELLOW_LED_OFF();
            }
        }


    }

}

// Inverse kinematics control
void double_joint_ik_control(UINT8* mn, UINT8 node1_id, UINT8 node2_id) {
//#define DOUBLE_IK_DEBUG // Define this to instead output stuff in terminal for debugging
//#define SKIP_END_EFFECTOR // Skip end effector for testing purposes
    static UINT32 start_millis = 0;
    static UINT8 has_end_effector = 0;
    float a1 = 0, a2 = 0;
    UINT8 ik_status = 0, i2c_status, command_type;

    switch(*mn) {
        case 1:
#if defined(END_EFFECTOR_PEN_ADDRESS) && !defined(DOUBLE_IK_DEBUG) && !defined(SKIP_END_EFFECTOR)
            if(PokeAddress(END_EFFECTOR_PEN_ADDRESS) == I2C_STATUS_SUCCESFUL) {
                putsUART1("A pen end effector was found and is used\n\r");
                has_end_effector = 1;
                retract_pen();
            }
#endif
#ifndef DOUBLE_IK_DEBUG
            if (disable_motor(node1_id) == I2C_STATUS_SUCCESFUL && disable_motor(node2_id) == I2C_STATUS_SUCCESFUL) {
                calibrate_encoder_zero(node1_id);
                calibrate_encoder_zero(node2_id);
                *mn = 2;
            } else {
                sprintf(buf,"One of the nodes %d or %d not found on the bus\n\r",node1_id, node2_id);
                putsUART1(buf);
                *mn = 0;

            }
#else
            *mn = 2;
#endif
            start_millis = millis;
            break;
        case 2:
            // Call for angles
            //ik_status = ik_circle(((float)(millis-start_millis))/1000.0,&command_type,&a1,&a2);
            ik_status = ik_path(((float)(millis-start_millis))/1000.0,&command_type,&a1,&a2);


            if (ik_status > 0) {
                sprintf(buf,"Couldn't reach the desired point (%d)\n\r",ik_status);
                putsUART1(buf);
            }

            switch(command_type) {
                case ELEMENT_TYPE_DEPLOY_END_EFFECTOR:
#if defined(END_EFFECTOR_PEN_ADDRESS) && !defined(DOUBLE_IK_DEBUG) && !defined(SKIP_END_EFFECTOR)
                    if (has_end_effector) deploy_pen();
#endif
                    break;
                case ELEMENT_TYPE_RETRACT_END_EFFECTOR:
#if defined(END_EFFECTOR_PEN_ADDRESS) && !defined(DOUBLE_IK_DEBUG) && !defined(SKIP_END_EFFECTOR)
                    if (has_end_effector) retract_pen();
#endif
                    break;
                case ELEMENT_TYPE_MOVE_TO_POS:
                case ELEMENT_TYPE_INTERPOLATE_POS:
                case ELEMENT_TYPE_SET_ANGLES:
#ifndef DOUBLE_IK_DEBUG
                    i2c_status = set_angle(node1_id,a1);
                    i2c_status = set_angle(node2_id,a2);
#endif
                    break;
                case ELEMENT_TYPE_EXIT:
                    *mn = 3;
                    break;
            }
#ifdef DOUBLE_IK_DEBUG
                // To terminal
                sprintf(buf,"%f, %f, %f\n\r",((float)(millis-start_millis))/1000.0, RAD_TO_DEG(a1), RAD_TO_DEG(a2));
                putsUART1(buf);
#endif
            break;
        case 3:
#if defined(END_EFFECTOR_PEN_ADDRESS) && !defined(DOUBLE_IK_DEBUG) && !defined(SKIP_END_EFFECTOR)
            if (has_end_effector) retract_pen();
            has_end_effector = 0;
#endif
#ifndef DOUBLE_IK_DEBUG
            set_calibration_status_unknown(node1_id);
            set_calibration_status_unknown(node2_id);
#endif
            *mn = 0;
            break;
    }
}

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