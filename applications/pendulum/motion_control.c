#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <xc.h>
#include <math.h>
#include "main_declarations.h"
#include "motion_control.h"
#include "mpu9150.h"
#include "../../api/pic32/modular_arms.h" //path to modular_arms.h

// Control states
#define CONTROL_STATE_PASSIVE 0
#define CONTROL_STATE_SWITCHING 1
#define CONTROL_STATE_ACTIVE 2

// Activation angles
#define SWITCH_TO_ACTIVE_ACTIVATION_TIME (2*FS)
#define TH2_ACTIVATION_ANGLE 0.1
#define TH2_DEACTIVATION_ANGLE (20.0*M_PI/180.0)
#define TH1_DEACTIVATION_ANGLE (90.0*M_PI/180.0)

UINT8 control_state = CONTROL_STATE_PASSIVE;
UINT16 switch_count = 0;

void disable_control() {
    set_calibration_status_unknown(NODE_ID);
    control_state = CONTROL_STATE_PASSIVE;
}

void control_logic(float th1, float th2, imu_store_struct* imu_data) {
    static float output;
    static UINT32 cnt;
    UINT8 mtr_i2c_status;


    // State driven
    switch (control_state) {
        case CONTROL_STATE_PASSIVE:
            YELLOW_LED_OFF();
            if (fabsf(th2) < TH2_ACTIVATION_ANGLE) {
                control_state = CONTROL_STATE_SWITCHING;
                switch_count = 0;
            }
            break;
        case CONTROL_STATE_SWITCHING:
            if ( (switch_count % (FS/10)) == 0) {
                YELLOW_LED_SWAP();
            }
            if (switch_count >= SWITCH_TO_ACTIVE_ACTIVATION_TIME ) {
                YELLOW_LED_ON();
                LED5_ON();
                delay_us(100);
                mtr_i2c_status = calibrate_encoder_zero(NODE_ID);
                if (mtr_i2c_status == I2C_STATUS_SUCCESFUL) {
                    sprintf(buf,"ACTIVATING MOTOR (FS = %d)\n\r",FS);
                    putsUART1(buf);
                    putsUART1("th1,th2,output,rate\n\r");
                    //reference = PID_feedback_loop(0.0,th2,&ref_th2,1); // Resets pid loop
                    output = state_feedback_loop(0.0,th2,0.0,imu_data,1); // Resets control loop
                    th1 = 0;
                    control_state = CONTROL_STATE_ACTIVE;
                } else {
                    sprintf(buf,"NO MOTOR FOUND (code: %d)\n\r",mtr_i2c_status);
                    putsUART1(buf);
                    control_state = CONTROL_STATE_PASSIVE;
                }
                LED5_OFF();
            }
            if (fabsf(th2) > TH2_ACTIVATION_ANGLE) {
                YELLOW_LED_OFF();
                control_state = CONTROL_STATE_PASSIVE;
            }
            switch_count++;
            break;
        case CONTROL_STATE_ACTIVE:
            // CONTROL ALGORITHM
            //reference = PID_feedback_loop(th1,th2,&ref_th2,0);
            output = state_feedback_loop(th1,th2,0.0,imu_data,0);
            LED5_ON(); PULSE_TRIGGER();
            //mtr_i2c_status = set_angular_velocity(NODE_ID, output);
            //mtr_i2c_status = set_torque(NODE_ID, output);
            mtr_i2c_status = set_voltage(NODE_ID, output);
            LED5_OFF();

            // Exit state if any of these conditions are fulfilled
            if (mtr_i2c_status != I2C_STATUS_SUCCESFUL || fabsf(th2) > TH2_DEACTIVATION_ANGLE || fabsf(th1) > TH1_DEACTIVATION_ANGLE) {

                // disable motor
                LED5_ON();
                set_calibration_status_unknown(NODE_ID);
                LED5_OFF();
                control_state = CONTROL_STATE_PASSIVE;
                YELLOW_LED_OFF();
                putsUART1("QUITTING CONTROL\n\r");
            }
            break;
    } // switch(control_state)

    // Print info every now and then
    if ( (control_state == CONTROL_STATE_ACTIVE && (FS < 170) ) ||\
            (control_state != CONTROL_STATE_ACTIVE && (cnt % (FS/4) == 0))  ) {
        //sprintf(buf,"th1: %0.2f deg, th2: %0.2f deg, out: %0.2f (%0.2fg)\n\r",\
        RAD_TO_DEG(th1), RAD_TO_DEG(th2),output,get_abs_acc(&imu_data) );
        sprintf(buf,"%0.2f, %0.2f, %0.2f, %0.2f\n\r",\
                RAD_TO_DEG(th1),RAD_TO_DEG(th2),output,get_control_signal(imu_data));
        putsUART1(buf);
    }

    if (imu_data->overflow) RED_LED_ON();
    //else RED_LED_OFF();

    cnt++;
}


float state_feedback_loop(float th1, float th2, float r, imu_store_struct* imu_data, UINT8 reset) {
    // state feedback parameters: states = [th1,th2,th1_dot,th2_dot]
    const float L[] = {-1.0000, 82.1998, -10.9891, 15.5814};
    const float Nr = -1.0000;

    static float th1_old = 0, th2_old = 0;
    th1 = -1*th1; // Fixes signs

    //th2 = 0.0;

    if (reset){
        th1_old = th1;
        th2_old = th2;
        return 0.0;
    }
    float th1_dot = (th1 - th1_old)/TS;
    float th2_dot = get_control_signal(imu_data);//(th2 - th2_old)/TS;
    th1_old = th1;
    th2_old = th2;

    return -1.0*(-1.0*(L[0]*th1 + L[1]*th2 + L[2]*th1_dot + L[3]*th2_dot) + Nr*r);
}

// Depreceted
#define sign(f) (((float)(f > 0)) - ((float)(f < 0)))
float PID_feedback_loop(float th1, float th2, float* rth2, UINT8 reset) {
    // PID-constants for keeping the Pendulum at a certain position
    const float P_pos = 0.0, I_pos = 0.0, D_pos = 0.01, N_pos = 100;
    const float P_pos_limit = 2.0*M_PI/180.0, I_pos_limit = 2.0*M_PI/180.0;
    static float th1_ref = 0.0, err_pos, err_cum_pos = 0.0, old_pos_err = 0.0;
    static float P_pos_out = 0, I_pos_out = 0, D_pos_out = 0;

    // PID-constants for keeping the pendulum straight up
    const float P_pend = 30.0, I_pend = 0.0, D_pend = 4.0, N_pend = 100;
    //const float P_pend = 0.0, I_pend = 0.0, D_pend = 0.0, N_pend = 10;
    // Working variables for pendulum balancing
    static float th2_ref = 0.0, err_pend, err_cum_pend = 0.0, old_th2 = 0.0;
    static float D_pend_out = 0; // Filtered D-part

    if (reset) {
        // pos pid
        th1_ref = 0.0;
        err_pos = 0.0;
        err_cum_pos = 0.0;
        old_pos_err = 0.0;
        P_pos_out = 0;
        I_pos_out = 0;
        D_pos_out = 0;

        // pend pid
        D_pend_out = 0.0;
        th2_ref = 0.0;
        err_pend = 0.0;
        err_cum_pend = 0.0;
        old_th2 = th2;

        *rth2 = th2_ref;
        return 0.0;
    }

    // Position control
    err_pos = (th1_ref - th1);
    err_cum_pos += err_pos*TS;
    P_pos_out = P_pos*err_pos;
    if (fabsf(P_pos_out) > P_pos_limit) P_pos_out = sign(P_pos_out)*P_pos_limit;
    I_pos_out = I_pos*err_cum_pos;
    if (fabsf(I_pos_out) > I_pos_limit) I_pos_out = sign(I_pos_out)*I_pos_limit;
    D_pos_out = (D_pos*N_pos*(err_pos - old_pos_err) + D_pos_out)/(N_pos*TS + 1);
    th2_ref = P_pos_out + I_pos_out + D_pos_out;
    old_pos_err = err_pos;

    // Pendulum balancing
    err_pend = (th2 - th2_ref);
    err_cum_pend += err_pend*TS;
    if (N_pend < 99)
        D_pend_out = (D_pend*N_pend*(th2 - old_th2) + D_pend_out)/(N_pend*TS + 1);
    else
        D_pend_out = D_pend*(th2 - old_th2)/TS;
    float output = P_pend*err_pend + I_pend*err_cum_pend + D_pend_out;// D_pend*(th2 - old_th2)/TS;
    old_th2 = th2;
    *rth2 = th2_ref;
    return output;
}