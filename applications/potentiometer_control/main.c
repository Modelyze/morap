/* 
 * File:   main.c
 * Author: Viktor Kozma
 *
 * Created on January 17, 2016, 3:19 PM
 *
 * Directly controls up to two motor nodes
 * by potentiometers.
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
 * To initiate control of the motors press BUTTON 1
 * and control the first motor with potentiometer 1 and 
 * control the second motor with potentiometer 2.
 * 
 * To change the id's of the controlled motors change the 
 * NODE1_ID and/or NODE2_ID definitions below.
 */
 
#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <xc.h>
#include "../../api/pic32/modular_arms.h" //path to modular_arms.h

// Configs
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1
#define SYS_FREQ (80000000L)

// Id's of the motors to be controlled
#define NODE1_ID 1
#define NODE2_ID 2

// Position or velociy feedback
//#define MODE_SPEED_FEEDBACK
//#define MODE_POSITION_FEEDBACK
#define MODE_DIRECT_VOLTAGE

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

// Pulse trigger for oscilloscope triggering
#define INIT_PULSE_TRIGGER() (TRISECLR = 1)
#define PULSE_TRIGGER() (LATESET = 1, LATECLR = 1)

#define A0 0
#define A1 1
unsigned int read_adc(unsigned char channel) {
    if (channel == A1)
        AD1CHS = ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN4;
    else if (channel == A0)
        AD1CHS = ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN2;
    else
        return 0;
    int i;
    for (i = 0; i < 10; i++); // small delay (may not be needed but w/e)
    AcquireADC10();
    for (i = 0; i < 100; i++); // Wait for sampling to complete
    ConvertADC10();
    while(!BusyADC10());
    return ReadADC10(0);
}

// Global vars
unsigned char buf[256]; // OUTPUT BUFFER FOR UART



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
    //Open Timer1, 256 Prescaler -> Fclock = 312500 Hz. With period = 6250 -> tick = 50 Hz
    OpenTimer1(T1_ON | T1_PS_1_256 | T1_SOURCE_INT, 6250);
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

    // configure and enable the ADC
    // A0 = AN2, A1 = AN4
    CloseADC10();    // ensure the ADC is off before setting the configuration
    AD1CHS = ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN2; // configure to sample AN2
    mPORTBSetPinsAnalogIn(SKIP_SCAN_ALL);
    AD1CSSL = ~(ENABLE_AN2_ANA | ENABLE_AN4_ANA);
    AD1CON3 = ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_16Tcy;
    AD1CON2 = ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | \
              ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF;
    AD1CON1 = ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_MANUAL | ADC_AUTO_SAMPLING_OFF;
    EnableADC10(); // Enable the ADC

    // Button and LEDS
    INIT_LEDS();
    LED4_OFF(); LED5_OFF();
    INIT_SHIELD_LEDS();
    RED_LED_OFF(); YELLOW_LED_OFF();
    INIT_PULSE_TRIGGER(); 
    INIT_BUTS();


    // SAYS HELLO
    putsUART1("HELLO\n\r");
    
    // INIT the arm
    init_arm(GetPeripheralClock());

    // Enables interrupts
    INTEnableSystemMultiVectoredInt();
    
    while(1){
        // Works only with interrupts
    }
    return (EXIT_SUCCESS);
}

// Timer 1 interuppt routine, 50 Hz
void __ISR(_TIMER_1_VECTOR, IPL2AUTO) _Timer1Handler(void) {
    static UINT32 cnt = 0; // Incremented variables used for timing

    // Motor status variables
    static UINT8 motor1_status = MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED;
    static UINT8 motor2_status = MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED;
    static UINT8 motor1_i2c_status = 1, motor2_i2c_status = 1;

    // Check if button 1 is pressed, if enable/disable motor 1
    static UINT8 but1_pressed = 0;
    static UINT32 prev_but1_cnt = 0;
    if (GET_BUT1() && !but1_pressed && (cnt-prev_but1_cnt > 5) ) {
        // Init/exit motor 1
        if (motor1_i2c_status == I2C_STATUS_SUCCESFUL) {
            if( motor1_status != MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED) {
                set_calibration_status_unknown(NODE1_ID);
                putsUART1("Motor 1 disabled\n\r");
            } else {
                calibrate_encoder_zero(NODE1_ID);
                putsUART1("Motor 1 enabled\n\r");
            }
        } else {
            sprintf(buf,"Motor 1 with id %d not found on the bus\n\r",NODE1_ID);
            putsUART1(buf);
        }
        // Debouncing and making sure the button only is pressed once
        // so holding it down doesn't call this function repeatedly
        prev_but1_cnt = cnt;
        but1_pressed = 1;
    } else if (!GET_BUT1() && but1_pressed && (cnt-prev_but1_cnt > 5)) {
        but1_pressed = 0;
        prev_but1_cnt = cnt;
    }
    // Check if button 2 is pressed, if enable/disable motor 2
    static UINT8 but2_pressed = 0;
    static UINT32 prev_but2_cnt = 0;
    if (GET_BUT2() && !but2_pressed && (cnt-prev_but2_cnt > 5) ) {
        // Init/exit motor 1
        if (motor2_i2c_status == I2C_STATUS_SUCCESFUL) {
            if( motor2_status != MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED) {
                set_calibration_status_unknown(NODE2_ID);
                putsUART1("Motor 2 disabled\n\r");
            } else {
                calibrate_encoder_zero(NODE2_ID);
                putsUART1("Motor 2 enabled\n\r");
            }
        } else {
            sprintf(buf,"Motor 2 with id %d not found on the bus\n\r",NODE2_ID);
            putsUART1(buf);
        }
        // Debouncing and making sure the button only is pressed once
        // so holding it down doesn't call this function repeatedly
        prev_but2_cnt = cnt;
        but2_pressed = 1;
    } else if (!GET_BUT2() && but2_pressed && (cnt-prev_but2_cnt > 5)) {
        but2_pressed = 0;
        prev_but2_cnt = cnt;
    }


    // Read motor status
    LED5_ON();
    motor1_i2c_status = get_status(NODE1_ID,&motor1_status);
    motor2_i2c_status = get_status(NODE2_ID,&motor2_status);
    LED5_OFF();

    // Send references to motors if they're enabled
    float send_value;
    if (motor1_i2c_status == I2C_STATUS_SUCCESFUL && motor1_status != MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED) {
#if defined(MODE_SPEED_FEEDBACK)
        send_value = (((float)read_adc(A0))-512)*6.2831/512; // +- 360 deg/s
        motor1_i2c_status = set_angular_velocity(NODE1_ID,send_value);
#elif defined(MODE_DIRECT_VOLTAGE)
        send_value = (((float)read_adc(A0))-512)*24/512; // +- 24V
        motor2_i2c_status = set_voltage(NODE1_ID,send_value);
#else
        send_value = (((float)read_adc(A0))-512)*1.5708/512; // +- 90 degrees
        motor1_i2c_status = set_angle(NODE1_ID,send_value);
#endif
        RED_LED_ON();
    } else RED_LED_OFF();
    if (motor2_i2c_status == I2C_STATUS_SUCCESFUL && motor2_status != MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED) {
#if defined(MODE_SPEED_FEEDBACK)
        send_value = (((float)read_adc(A1))-512)*6.2831/512; // +- 360 deg/s
        motor2_i2c_status = set_angular_velocity(NODE2_ID,send_value);
#elif defined(MODE_DIRECT_VOLTAGE)
        send_value = (((float)read_adc(A1))-512)*24/512; // +- 24V
        motor2_i2c_status = set_voltage(NODE2_ID,send_value);
#else
        send_value = (((float)read_adc(A1))-512)*1.5708/512; // +- 90 degrees
        motor2_i2c_status = set_angle(NODE2_ID,send_value);
#endif
        YELLOW_LED_ON();
    } else YELLOW_LED_OFF();

    // Blink one on board led every 2 sec for visual feedback
    if (cnt % 100 == 0) {
        LED4_SWAP();
    }
    cnt++;
    mT1ClearIntFlag(); // clear interrupt flag
}











