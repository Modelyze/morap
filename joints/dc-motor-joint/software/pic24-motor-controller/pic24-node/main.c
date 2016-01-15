/*
 * File:   main.c
 * Author: Viktor Kozma
 *
 * This program controls a motor node for a
 * modular robotic arm. It includes encoder
 * reading, motor control and communication
 * with a master controller through I2C.
 *
 * Important!
 * This code requires specific data to be
 * programmed into the microcontroller internal
 * eeprom memory.
 *
 * In addition, to accomodate for this fact the
 * programmer configuration must be set to
 * preserve the internal eeprom. This is done
 * in MPLABX for the PICKIT3 by:
 * Run -> Set Project Configuration -> Customize ->
 * -> PICKIT3 -> Conserve EEPROM Memory
 *
 *
 * Created on April 28, 2015
 */

//#include <PIC24F_plib.h>
#include <xc.h>

// Libraries
#include "main_init_funcs.h"
#include "LMD18200_drivers.h"
#include "pic24_i2c_slave.h"
#include "eeprom_storage.h"
#include "control_tuner.h"

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
#pragma config OSCIOFNC = ON            // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)

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

// START OF PROGRAM ------------------------------------------------------------
// Motor id
unsigned char motor_id = 0;

// Status
system_status_struct systemStatus = {
    .enc_calib = ENCODER_CALIBRATION_NEEDED,
    .control_prog_status = CONTROL_PROG_STATUS_NOTHING,
    .new_control_available = NO_NEW_CONTROLLER_AVAILABLE,
    .control_mode = CONTROL_MODE_DISABLED
};

// controller variables passed between
volatile float pos_ref = 0, vel_ref = 0, drive_voltage = 0;
control_params_struct posControlParams, velControlParams;
motor_params_struct motorData;

#ifdef LED1_MODE_I2C
volatile int led1_timer_count = 0;
#endif

int main(void) {

    // 8 MHz x 4 PLL from FRC
    CLKDIVbits.RCDIV = 0;
    CLKDIVbits.DOZEN = 0;

    INIT_LEDS();
    LED1_OFF();
    LED2_OFF();

    // Read from EEPROM, this is done first before anything else is
    // initiated in order to remove chances of data corruption
    if (!EepVerifyTest()) eepromErrorLoop();
    motor_id = EepReadNodeID();
    EepReadMotorParams(&motorData);
    EepReadPosControlParams(&posControlParams);
    EepReadVelControlParams(&velControlParams);
    if (velControlParams.cMode != CMODE_SPEED_CONTROL || posControlParams.cMode != CMODE_POSITION_CONTROL || motor_id > 16)
        eepromErrorLoop();

    Nop(); // Breakpoints for debugging
    Nop();

    // Initiate all the functions!
    init_encoders();
    init_timers(DEFAULT_FS);
    init_motor();
    init_i2c(motor_id);

    MOTOR_DISABLE();

    while(1) {
        // Do nothing, only works with interrupts
    }

    return 0;
}

// Prototypes, implementation at bottom
void control_loop(control_params_struct*,float,unsigned char);

// Timer interrupt for sampling
//#define USE_LP_ON_DRIVE_VOLTAGE
void _ISR _T1Interrupt(void) {
    if(systemStatus.enc_calib == ENCODER_IS_CALIBRATED) LED2_ON();

    static unsigned int blink_count = 0;   // Count used for blinking the enc_needed thing
    static unsigned char reset_control_loop = 0; // If 1 the control loop will reset
    static unsigned char prev_control_mode = CONTROL_MODE_DISABLED;
    static unsigned int Fs = DEFAULT_FS;
#ifdef USE_LP_ON_DRIVE_VOLTAGE
    static float input_drive_voltage = 0.0; // Low pass filter for drive voltage
#endif

    // Update eventual stored motor speed
    update_motor_speed();

    // If new control parameters is found, update
    if (systemStatus.new_control_available != NO_NEW_CONTROLLER_AVAILABLE) {
        if (systemStatus.new_control_available == NEW_CONTROLLER_AVAILABLE) {
            if (newControlParams.cMode == CMODE_POSITION_CONTROL)
                posControlParams = newControlParams;
            else if (newControlParams.cMode == CMODE_SPEED_CONTROL)
                velControlParams = newControlParams;
        } else if (systemStatus.new_control_available == NEW_CONTROLLER_DEFAULT) {
            EepReadPosControlParams(&posControlParams);
            EepReadVelControlParams(&velControlParams);
        }        
        systemStatus.new_control_available = NO_NEW_CONTROLLER_AVAILABLE;
        prev_control_mode = CONTROL_MODE_NEVER; // Force update of sampling rate
    }

    // Sets the sampling rate if the control mode is changed
    if (systemStatus.control_mode != prev_control_mode) {
        if (systemStatus.control_mode == CONTROL_MODE_POSITION_FEEDBACK)
            Fs = posControlParams.Fs;
        else if (systemStatus.control_mode == CONTROL_MODE_SPEED_FEEDBACK)
            Fs = velControlParams.Fs;
        else
            Fs = DEFAULT_FS;
        reset_control_loop = 1;
        blink_count = 0;
#ifdef LED1_MODE_I2C
        led1_timer_count = -1;
        LED1_OFF();
#endif
#ifdef USE_LP_ON_DRIVE_VOLTAGE
        input_drive_voltage = 0.0;
#endif
        set_timer1_period(Fs);
        TMR1 = 0;
        prev_control_mode = systemStatus.control_mode;
    }
    

    // Does stuff depending on what it wants to do
    if (systemStatus.control_mode == CONTROL_MODE_DISABLED || systemStatus.enc_calib == ENCODER_CALIBRATION_NEEDED) {
        MOTOR_DISABLE();
        MOTOR_BRAKE_ON();
        set_motor_speed(0.0);
        reset_control_loop = 1;
#ifdef USE_LP_ON_DRIVE_VOLTAGE
        input_drive_voltage = 0.0;
#endif
    } else if (systemStatus.control_mode == CONTROL_MODE_POSITION_FEEDBACK) {
        MOTOR_ENABLE();
        MOTOR_BRAKE_OFF();
        control_loop(&posControlParams,pos_ref,reset_control_loop); // Main control loop
        reset_control_loop = 0;
    } else if (systemStatus.control_mode == CONTROL_MODE_SPEED_FEEDBACK) {
        MOTOR_ENABLE();
        MOTOR_BRAKE_OFF();
        control_loop(&velControlParams,vel_ref,reset_control_loop); // Main control loop
        reset_control_loop = 0;
    } else if (systemStatus.control_mode == CONTROL_MODE_VOLTAGE) {
        MOTOR_ENABLE();
        MOTOR_BRAKE_OFF();
#ifdef USE_LP_ON_DRIVE_VOLTAGE
#define DRIVE_DIGITAL_LP_CONSTANT     (1.0/((float)Fs)/0.05)
        input_drive_voltage = (1-DRIVE_DIGITAL_LP_CONSTANT)*input_drive_voltage + DRIVE_DIGITAL_LP_CONSTANT*drive_voltage;
        set_motor_speed(input_drive_voltage);
#else
        set_motor_speed(drive_voltage);
#endif
        reset_control_loop = 1;
    } else {
        // Just in case
        systemStatus.control_mode = CONTROL_MODE_DISABLED;
        reset_control_loop = 1;
    }

    // LED CONTROL
    if (systemStatus.enc_calib == ENCODER_CALIBRATION_NEEDED) {
        // When encoder calibration is needed, 1Hz 20% signal
        float blink_sec = ((float) blink_count)/((float) Fs);
        if ( (blink_sec > 0.8) && (blink_sec < 2.5) ) {
            LED2_ON();
            blink_count = 3*Fs;
        } else if (blink_sec > 3.2) {
            LED2_OFF();
            blink_count = 0;
        }
        blink_count += 1;
    } else {
        // When running, on all the time if thermal flag is on
        // else switch it off, this allows the user to see
        // CPU utilization
        if (!GET_THERMAL_FLAG())
            LED2_ON();
        else
            LED2_OFF();
    }

#ifdef LED1_MODE_I2C
    if (led1_timer_count < 10000 && led1_timer_count >= ((int)(LED1_MODE_I2C_FLASH_TIME*((float)Fs))) ){
        LED1_OFF();
        led1_timer_count = 20000 - ((int)(0.5*LED1_MODE_I2C_FLASH_TIME*((float)Fs)));
    } else if (led1_timer_count >= 20000){
        led1_timer_count = -1;
    } else if (led1_timer_count >= 0) {
        led1_timer_count++;
    }
#endif

    IFS0bits.T1IF = 0;      // Interrupt flag
}

#define USE_LP_ON_CURRENT_LIMITER
#define CURRENT_LIMITER_DLPFC 0.2 // -> tau = 4*Ts
//#define IGNORE_CURRENT_LIMITER_WITH_INTEGRATOR
void control_loop(control_params_struct* cp, float reference, unsigned char reset_old_params) {
    static float old_y[4] = {0}, old_r[4] = {0};
    static float old_uff[4] = {0}, old_ufb[4] = {0};
    static float I_ff = 0, I_fb = 0;
    static float u_nosat = 0, old_theta = 0;
    static unsigned char issat = 0;
#ifdef USE_LP_ON_CURRENT_LIMITER
    static float old_spd = 0;
#endif
    unsigned char i; // loop var
    if (reset_old_params) {
        float th_curr = ENCODER_TO_RAD(motorData.ppr)*encoder_value/motorData.n;
        for (i = 0; i < 4; i++) {
            if (cp->cMode == 0)
                old_y[i] = th_curr;
            else
                old_y[i] = 0;
            old_r[i] = reference;
            old_uff[i] = 0;
            old_ufb[i] = 0;
        }
        issat = 0;
        old_theta = th_curr;
        I_ff = 0;
        I_fb = 0;
        save_motor_speed(0);
    }

    //update_motor_speed();

    float theta = ENCODER_TO_RAD(motorData.ppr)*encoder_value/motorData.n;

    if (cp->cMode == CMODE_POSITION_CONTROL)
        old_y[0] = theta;
    else
        old_y[0] = (theta - old_theta)/(1.0/((float)cp->Fs));
    old_r[0] = reference;

    float u_fb = 0, u_ff = 0;
    for (i = 0; i < cp->nd; i++) { // Feed back
        u_fb += cp->d[i]*old_y[(cp->nd - 1 - i)];
    }
    for (i = 0; i < cp->nc; i++) { // Feed forward
        u_ff += cp->c[i]*old_r[(cp->nc - 1 - i)];
    }
    for (i = 0; i < cp->nf; i++) {
        u_fb -= cp->f[i]*old_ufb[(cp->nf - 1 - i)];
        u_ff -= cp->f[i]*old_uff[(cp->nf - 1 - i)];
    }
    // Integrator part with antiwindup
    if (cp->I > 0 && issat == 0) {
        I_ff = I_ff + cp->I*old_r[0];
        I_fb = I_fb + cp->I*old_y[0];
    }

    u_nosat = (u_ff + I_ff) - (u_fb + I_fb); // non-saturated output

    // Output voltage limiting for anti windup purposes
    float u = u_nosat;
    if (u > LIMIT_VOLTAGE) u = LIMIT_VOLTAGE;
    else if (u < -LIMIT_VOLTAGE) u = -LIMIT_VOLTAGE;

    // Current limiting, with optional low pass filter on speed
#ifdef IGNORE_CURRENT_LIMITER_WITH_INTEGRATOR
    if (controlParams.I == 0) {
#endif
        float spd = (theta - old_theta)/(1.0/((float)cp->Fs));
    #ifdef USE_LP_ON_CURRENT_LIMITER
        spd = CURRENT_LIMITER_DLPFC*spd + (1-CURRENT_LIMITER_DLPFC)*old_spd;
        old_spd = spd;
    #endif
        float temp1 = motorData.K * motorData.n * spd;
        float temp2 = motorData.R * motorData.imax;
        if (u > (temp1 + temp2) ) u = (temp1 + temp2);
        else if (u < (temp1 - temp2) ) u = (temp1 - temp2);
#ifdef IGNORE_CURRENT_LIMITER_WITH_INTEGRATOR
    }
#endif

    if (u != u_nosat) issat = 1; else issat = 0;

    save_motor_speed(u);

    for(i = (cp->nd - 1); i > 0; i--) old_y[i] = old_y[i-1];
    for(i = (cp->nc - 1); i > 0; i--) old_r[i] = old_r[i-1];
    for(i = (cp->nf - 1); i > 0; i--) {
        old_ufb[i] = old_ufb[i-1];
        old_uff[i] = old_uff[i-1];
    }
    old_ufb[0] = u_fb;
    old_uff[0] = u_ff;
    old_theta = theta;
}