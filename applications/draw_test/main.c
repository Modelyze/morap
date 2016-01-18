/*
 * File:   main.c
 * Author: Viktor Kozma
 *
 * Created on January 18, 2016, 2:13 PM
 *
 * Does a feedback control sequence in which up
 * to two motors repeatedly goes between three
 * set points. If an end-effector is present on
 * the bus it will be used once during each set
 * point.
 *
 * Electrical connections (uC loc(uno32 pin)):
 * On board uno32 leds on PORTG6 and PORTF0
 * RED LED ON PORTD1(5)
 * YELLOW LED ON PORTD2(6)
 * BUTTON 1 on PORTD8(2)
 * BUTTON 2 on PORTD9(7)
 *
 * To initiate and exit the control sequence
 * press button 2.
 *
 * To change the id's of the controlled motors 
 * change the NODE1_ID and/or NODE2_ID
 * definitions below. The parameters of the
 * control sequence can also be changed.
 */

#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <xc.h>
#include "../../api/pic32/modular_arms.h" //path to modular_arms.h
#include "../../end-effectors/pen/drivers/pen_end_effector.h" // pen end effector

// Configs
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#define SYS_FREQ (80000000L)

// Id's of the motors to be controlled
#define NODE1_ID 1
#define NODE2_ID 2

// Control sequence definitions
#define MIN_CONTROL_VALUE   -0.52   // rads
#define MAX_CONTROL_VALUE   0.52    // rads
#define CONTROL_WAIT_TIME   1500    // ms, > SERVO_DEPLOY_DELAY + SERVO_DEPLOY_TIME
#define SERVO_DEPLOY_DELAY  600     // ms
#define SERVO_DEPLOY_TIME   500     // ms

// Clock reading functions used with the above config
#define GetSystemClock()           (SYS_FREQ)
#define GetPeripheralClock()       (SYS_FREQ/1)
#define GetInstructionClock()      (SYS_FREQ)

// ONBOARD LEDS OF THE UNO32
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

// Global vars
unsigned char buf[256]; // OUTPUT BUFFER FOR UART
volatile UINT32 millis = 0; // Increments every 1/1000 sec

// prototype
void control_test(UINT8*,UINT8,UINT8);

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

    // Button and LEDS
    INIT_LEDS();
    LED4_OFF(); LED5_OFF();
    INIT_SHIELD_LEDS();
    RED_LED_OFF(); YELLOW_LED_OFF();
    INIT_BUTS();

    // SAYS HELLO
    putsUART1("HELLO\n\r");

    // INIT the arm
    init_arm(GetPeripheralClock());

    // Enables interrupts
    INTEnableSystemMultiVectoredInt();

    UINT8 mode_engaged = 0; // 0 = off, 1 = init, 2 = run, 3 = shut down
    UINT8 but2_pressed = 0;
#define BUT_DEBOUNCE 10 // ms
#define SAMPLE_DELAY 20 // ms
    UINT32 prev_millis_but2 = 0, prev_millis = 0; // Timing
    while(1){

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
            control_test(&mode_engaged,NODE1_ID, NODE2_ID);
            if (mode_engaged == 0) {
                putsUART1("DISENGAGED!\n\r");
                YELLOW_LED_OFF();
            }
        }


    }

}

// Does a simple min, middle, max control sequence. If a pen end effector is
// present on the bus it will use it.
void control_test(UINT8* mn, UINT8 node1_id, UINT8 node2_id) {
    static unsigned long int prev_swap, cycle_count; // millis
    static UINT8 has_end_effector = 0, has_motor_1 = 0, has_motor_2 = 0, control_state = 0;
    static UINT8 min_max_mid = 0; // min = 0, mid_going_max = 1, max = 2, 3 = mid_going_min

    switch(*mn) {
        case 1: // INIT
            has_end_effector = 0;
            has_motor_1 = 0;
            has_motor_2 = 0;

            // Check if there's a valid end effector on the bus
#ifdef END_EFFECTOR_PEN_ADDRESS
            if(retract_pen() == I2C_STATUS_SUCCESFUL) {
                putsUART1("A pen end effector was found and is used\n\r");
                has_end_effector = 1;
            }
#endif
            // Check which motors are present on the bus
            if (calibrate_encoder_zero(node1_id) == I2C_STATUS_SUCCESFUL) {
                set_angle(node1_id,0);
                has_motor_1 = 1;
                sprintf(buf,"Motor 1 with id %d found and is used\n\r",node1_id);
                putsUART1(buf);
            }
            if (calibrate_encoder_zero(node2_id) == I2C_STATUS_SUCCESFUL) {
                set_angle(node2_id,0);
                has_motor_2 = 1;
                sprintf(buf,"Motor 2 with id %d found and is used\n\r",node2_id);
                putsUART1(buf);
            }

            if (has_motor_1 == 0 && has_motor_2 == 0) {
                sprintf(buf,"Either of motors with ids %d and %d not found, quitting\n\r",node1_id,node2_id);
                putsUART1(buf);
                *mn = 0;
            } else {
                *mn = 2;
            }
            prev_swap = millis;
            control_state = 0;
            cycle_count = 0;
            break;
        case 2: // RUN

            switch(control_state) {
                case 0: // start
                    prev_swap = millis;
                    control_state = 1;
                    min_max_mid = 0;
                    if (has_motor_1) set_angle(node1_id,MIN_CONTROL_VALUE);
                    if (has_motor_2) set_angle(node2_id,MAX_CONTROL_VALUE);
                    break;
                case 1: // delay until servo is deployed
                    if (millis - prev_swap > SERVO_DEPLOY_DELAY) {
#ifdef END_EFFECTOR_PEN_ADDRESS
                        if (has_end_effector) deploy_pen();
#endif
                        control_state = 2;
                    }
                    break;
                case 2: // Servo deployed
                    if (millis - prev_swap > (SERVO_DEPLOY_DELAY + SERVO_DEPLOY_TIME)) {
#ifdef END_EFFECTOR_PEN_ADDRESS
                        if (has_end_effector) retract_pen();
#endif
                        control_state = 3;
                    }
                    break;
                case 3: // Servo retracted
                    if (millis - prev_swap > CONTROL_WAIT_TIME) {
                        if (min_max_mid == 0) { // Min
                            if (has_motor_1) set_angle(node1_id,0);
                            if (has_motor_2) set_angle(node2_id,0);
                            min_max_mid = 1;
                        } else if (min_max_mid == 1) { // Mid going max
                            if (has_motor_1) set_angle(node1_id,MAX_CONTROL_VALUE);
                            if (has_motor_2) set_angle(node2_id,MIN_CONTROL_VALUE);
                            min_max_mid = 2;
                        } else if (min_max_mid == 2) { // Max
                            if (has_motor_1) set_angle(node1_id,0);
                            if (has_motor_2) set_angle(node2_id,0);
                            min_max_mid = 3;
                        } else if (min_max_mid == 3) {
                            if (has_motor_1) set_angle(node1_id,MIN_CONTROL_VALUE);
                            if (has_motor_2) set_angle(node2_id,MAX_CONTROL_VALUE);
                            min_max_mid = 0;
                            cycle_count++;
                            if (cycle_count % 10 == 0) {
                                sprintf(buf,"%d cycles performed\n\r",cycle_count);
                                putsUART1(buf);
                            }
                        }
                        control_state = 1;
                        prev_swap = millis;
                    }
                    break;
            }
            break;
        case 3: // SHUT DOWN
#ifdef END_EFFECTOR_PEN_ADDRESS
            if (has_end_effector) retract_pen();
#endif
            has_end_effector = 0;
            if (has_motor_1) set_calibration_status_unknown(node1_id);
            has_motor_1 = 0;
            if (has_motor_2) set_calibration_status_unknown(node2_id);
            has_motor_2 = 0;
            *mn = 0;
            sprintf(buf,"Total cycles performed: %d\n\r",cycle_count);
            putsUART1(buf);
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

