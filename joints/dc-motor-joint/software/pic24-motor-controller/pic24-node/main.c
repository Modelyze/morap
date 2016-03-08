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
#include "main_init_functions.h"
#include "LMD18200_drivers.h"
#include "pic24_i2c_slave.h"
#include "eeprom_storage.h"
#include "control_tuner.h"


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
volatile float pos_ref = 0, vel_ref = 0, drive_voltage = 0, amp_ref = 0;
control_params_struct posControlParams, velControlParams;
motor_params_struct motorData;

#ifdef LED1_MODE_I2C
volatile int led1_timer_count = 0;
#endif

int main(void) {

    init_oscillator();

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
//        MOTOR_ENABLE_SWAP(); // Motor pin 5
//        int i;
//        for(i = 0; i < 10000; i++);
    }

    return 0;
}

// Prototypes, implementation at bottom
void control_loop(control_params_struct*,float,unsigned char);
void voltage_drive(float,unsigned char);
void set_current(float,unsigned char);

// Timer interrupt for sampling
void _ISR _T1Interrupt(void) {
    if(systemStatus.enc_calib == ENCODER_IS_CALIBRATED) LED2_ON();

    static unsigned int blink_count = 0;   // Count used for blinking the enc_needed thing
    static unsigned char reset_control_loop = 0; // If 1 the control loop will reset
    static unsigned char prev_control_mode = CONTROL_MODE_DISABLED;
    static unsigned int Fs = DEFAULT_FS;

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

    // Sets the sampling rate and resets parameters if the control mode is changed
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
        set_timer1_period(Fs);
        TMR1 = 0;
        prev_control_mode = systemStatus.control_mode;
    }
    

    // Does stuff depending on what it wants to do
    if (systemStatus.control_mode == CONTROL_MODE_DISABLED || systemStatus.enc_calib == ENCODER_CALIBRATION_NEEDED) {
        MOTOR_DISABLE(); MOTOR_BRAKE_ON();
        set_motor_speed(0.0);
    } else if (systemStatus.control_mode == CONTROL_MODE_POSITION_FEEDBACK) {
        MOTOR_ENABLE(); MOTOR_BRAKE_OFF();
        control_loop(&posControlParams,pos_ref,reset_control_loop); // Main pos control loop
        reset_control_loop = 0;
    } else if (systemStatus.control_mode == CONTROL_MODE_SPEED_FEEDBACK) {
        MOTOR_ENABLE(); MOTOR_BRAKE_OFF();
        control_loop(&velControlParams,vel_ref,reset_control_loop); // Main speed control loop
        reset_control_loop = 0;
    } else if (systemStatus.control_mode == CONTROL_MODE_VOLTAGE) {
        MOTOR_ENABLE(); MOTOR_BRAKE_OFF();
        voltage_drive(drive_voltage,reset_control_loop);
        reset_control_loop = 0;
    } else if (systemStatus.control_mode == CONTROL_MODE_DIRECT_VOLTAGE) {
        MOTOR_ENABLE(); MOTOR_BRAKE_OFF();
        set_motor_speed(drive_voltage);
    } else if (systemStatus.control_mode == CONTROL_MODE_CURRENT) {
        MOTOR_ENABLE(); MOTOR_BRAKE_OFF();
        set_current(amp_ref,reset_control_loop);
        reset_control_loop = 0;
    } else {
        // Just in case
        systemStatus.control_mode = CONTROL_MODE_DISABLED;
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
#ifdef USE_LP_ON_CURRENT_LIMITER
        old_spd = 0;
#endif
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
    float spd = (theta - old_theta)/(1.0/((float)cp->Fs));
#ifdef USE_LP_ON_CURRENT_LIMITER
    spd = CURRENT_LIMITER_DLPFC*spd + (1-CURRENT_LIMITER_DLPFC)*old_spd;
    old_spd = spd;
#endif
    float temp1 = motorData.K * motorData.n * spd;
    float temp2 = motorData.R * motorData.imax;
    if (u > (temp1 + temp2) ) u = (temp1 + temp2);
    else if (u < (temp1 - temp2) ) u = (temp1 - temp2);

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

#define DRIVE_DIGITAL_LP_CONSTANT     (1.0/((float)DEFAULT_FS)/0.05)
//#define USE_LP_FILTER_ON_VOLTAGE_CONTROL
#define USE_CURRENT_LIMITING_ON_VOLTAGE_CONTROL
void voltage_drive(float voltage, unsigned char reset){
#ifdef USE_LP_FILTER_ON_VOLTAGE_CONTROL
    static float input_drive_voltage = 0;
    if (reset) input_drive_voltage = 0;
    voltage = (1-DRIVE_DIGITAL_LP_CONSTANT)*input_drive_voltage + DRIVE_DIGITAL_LP_CONSTANT*voltage;
    input_drive_voltage = voltage;
#endif
#ifdef USE_CURRENT_LIMITING_ON_VOLTAGE_CONTROL
    static float old_theta = 0;
#ifdef USE_LP_ON_CURRENT_LIMITER_VOLTAGE
    static float old_spd = 0;
    if(reset) old_spd = 0;
#endif
    float theta = ENCODER_TO_RAD(motorData.ppr)*encoder_value/motorData.n;
    if(reset) old_theta = theta;
    float spd = (theta - old_theta)/(1.0/(DEFAULT_FS));
    old_theta = theta;
#ifdef USE_LP_ON_CURRENT_LIMITER_VOLTAGE
    spd = CURRENT_LIMITER_DLPFC*spd + (1-CURRENT_LIMITER_DLPFC)*old_spd;
    old_spd = spd;
#endif
    float temp1 = motorData.K * motorData.n * spd;
    float temp2 = motorData.R * motorData.imax;
    if (voltage > (temp1 + temp2) ) voltage = (temp1 + temp2);
    else if (voltage < (temp1 - temp2) ) voltage = (temp1 - temp2);
#endif
    
    set_motor_speed(voltage);
}

void set_current(float current, unsigned char reset) {
    static float old_theta = 0.0;
    float theta = ENCODER_TO_RAD(motorData.ppr)*encoder_value/motorData.n;
    if (reset == 1) old_theta = theta;
    if (current > motorData.imax) current = motorData.imax;
    else if (current < -motorData.imax) current = -motorData.imax;

    float u = motorData.K*motorData.n*(theta-old_theta)/(1.0/((float)DEFAULT_FS)) + motorData.R*current;
    old_theta = theta;
    set_motor_speed(u);
}