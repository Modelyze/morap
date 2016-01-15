#ifndef CONTROL_TUNER_C
#define	CONTROL_TUNER_C

//#include <PIC24F_plib.h>
#include <xc.h>
#include "control_tuner.h"


unsigned char tune_control_parameters(unsigned char type, float J, float w) {
    // Check for plausible input parameters
    if (J < 0 || J > 10 || w > 0 || w < -200) {
        return CONTROL_TUNING_FAILED;
    }

    unsigned char toReturn = CONTROL_TUNING_FAILED;
    // Globally used sampling time, TODO: make dynamic(ish)
    switch(type) {
        case PD_POSITION_CONTROLLER:
            toReturn = PD_position_control_parameters(&newControlParams,&motorData,J,w);
            break;
        case PID_POSITION_CONTROLLER:
            toReturn = PID_position_control_parameters(&newControlParams,&motorData,J,w);
            break;
        case PI_SPEED_CONTROLLER:
            toReturn = PI_speed_control_parameters(&newControlParams,&motorData,J,w);
            break;
    }
    return toReturn;
}

// Calculates relevant sampling frequencies depending on the fastest pole
//unsigned int calculate_Fs(float w) {
//  if (w < 0) w = w*-1;
//
//  unsigned int Fs;
//  // If cases approach, a more optimal solution should exist
//  if (w <= 15.0) Fs = 50;
//  else if (w <= 30.0) Fs = 100;
//  else if (w <= 50.0) Fs = 160;
//  else if (w <= 65.0) Fs = 200;
//  else if (w <= 80.0) Fs = 250;
//  else if (w <= 100.0) Fs = 320;
//  else if (w <= 125.0) Fs = 400;
//  else if (w <= 150.0) Fs = 500;
//  else if (w <= 250.0) Fs = 625;
//  else Fs = 800;
//
//  return Fs;
//}
// Doesn't really work since the provided inertia is often overestimated which
// means that the motor will underestimate the Fs needed to drive that load.
// Better just to use a high Fs and leave it at that

unsigned char PD_position_control_parameters(control_params_struct* cp, motor_params_struct* mtr, float J, float w) {
    
    // Motor transfer function parameters
    float g0 = mtr->eta*mtr->K*mtr->n/(mtr->R*J);
    float g1 = (mtr->eta*mtr->n*mtr->n*mtr->K*mtr->K+mtr->R*mtr->c)/(mtr->R*J);

    // w1 and w2 complex conjugate
    if (w < -g1) w = -g1; // If the controller is faster than the motor it looses performance
    float w_real = DOUBLE_POLE_REAL_FACTOR*w;
    float w_imag = DOUBLE_POLE_IMAG_FACTOR*w;
    float w_3 = -g1; // Third pole equal to fastest pole of the motor

    // Continuous time variables
    float w1w2 = (w_real*w_real + w_imag*w_imag);
    float r0 = -g1 - w_real - w_real - w_3;
    float s0 = -(w1w2*w_3)/g0;
    float s1 = (2*g1*w_real + g1*w_3 + w1w2 + 2*w_3*w_real + g1*g1)/g0;

    // Feed forward
    float t1 = w1w2/g0;
    //float t0 = -w_3*t1;

    // Discretization
    unsigned int Fs;
    if (w_3 < -70) Fs = 625; else Fs = 400;
    float Ts_d = 1/((float) Fs);

    // Tustin discretization method
    float mr = (Ts_d*r0 + 2.0), f0 = (Ts_d*r0-2.0)/mr;
    float c1 = (2*t1 - Ts_d*t1*w_3)/mr, c0 = -1*(2*t1 + Ts_d*t1*w_3)/mr;
    float d1 = (2*s1 + Ts_d*s0)/mr, d0 = (Ts_d*s0 - 2*s1)/mr;

    // Store data in return struct
    cp->cMode = CMODE_POSITION_CONTROL;
    cp->Fs = Fs;
    cp->nd = 2; cp->d[0] = d0; cp->d[1] = d1;
    cp->nc = 2; cp->c[0] = c0; cp->c[1] = c1;
    cp->nf = 1; cp->f[0] = f0;
    cp->I = 0;

    return CONTROL_TUNING_SUCCESFUL;
}

unsigned char PI_speed_control_parameters(control_params_struct* cp, motor_params_struct* mtr, float J, float w) {
    // Motor transfer function parameters
    float g0 = mtr->eta*mtr->K*mtr->n/(mtr->R*J);
    float g1 = (mtr->eta*mtr->n*mtr->n*mtr->K*mtr->K+mtr->R*mtr->c)/(mtr->R*J);

    if (w < -g1) w = -g1; // If the controller is faster than the motor it looses performance
    float w_1 = w;
    float w_2 = -g1, w_3 = -g1; // Same as the pole for the motor

    // Continuous time variables
    float r0 = -g1 - w_1 - w_2 - w_3;
    float s0 = -(w_1*w_2*w_3)/g0;
    float s1 = (g1*w_1 + g1*w_2 + g1*w_3 + w_1*w_2 + w_1*w_3 + w_2*w_3 + g1*g1)/g0;

    //Feedforward:
    float t2 = -w_1/g0, t1 = -t2*(w_2 + w_3), t0 = t2*w_2*w_3;

    // Seperate the integrator part so that
    // ufb = (s1-s0/r0)/(r0 + s) + (s0/r0)/s,
    // uff = (t2*s + t1-t0/r0)/(r0+s) + (s0/r0)/s
    float sf0 = s1 - s0/r0;
    float tf1 = t2, tf0 = t1-t0/r0;
    float If = s0/r0;

    // Discretization
    // prefer slighlty lower sampling rates as this improves the
    // resolution of the discretized speed
    unsigned int Fs;
    if (w_3 < -150) Fs = 625;
    else if (w_3 < -100) Fs = 400;
    else Fs = 320;
    float Ts_d = 1/((float) Fs);

    // Tustin method on seperated integrator version
    float mr = Ts_d*r0 + 2;
    float d1 = Ts_d*sf0/mr, d0 = Ts_d*sf0/mr;
    float c1 = (2*tf1 + Ts_d*tf0)/mr, c0 = (Ts_d*tf0 - 2*tf1)/mr;
    float f0 = (Ts_d*r0 - 2)/mr;
    float Id = If*Ts_d;

    // Store data in return struct
    cp->cMode = CMODE_SPEED_CONTROL;
    cp->Fs = Fs;
    cp->nd = 2; cp->d[0] = d0; cp->d[1] = d1;
    cp->nc = 2; cp->c[0] = c0; cp->c[1] = c1;
    cp->nf = 1; cp->f[0] = f0;
    cp->I = Id;

    return CONTROL_TUNING_SUCCESFUL;
}

unsigned char PID_position_control_parameters(control_params_struct* cp, motor_params_struct* mtr, float J, float w) {
    // Motor transfer function parameters
    float g0 = mtr->eta*mtr->K*mtr->n/(mtr->R*J);
    float g1 = (mtr->eta*mtr->n*mtr->n*mtr->K*mtr->K+mtr->R*mtr->c)/(mtr->R*J);

    // w1 and w2 complex conjugate
    if (w < -g1) w = -g1; // If the controller is faster than the motor it looses performance
    float w_real = DOUBLE_POLE_REAL_FACTOR*w;
    float w_imag = DOUBLE_POLE_IMAG_FACTOR*w;
    float w_3 = -g1, w_4 = -g1; // w_3 & w_4 same as the pole for the motor in open loop

    // Continuous time variables
    float w1w2 = (w_real*w_real + w_imag*w_imag);
    float r0 = -g1 - 2*w_real - w_3 - w_4;
    float s0 = (w1w2*w_3*w_4)/g0;
    float s1 = -(w1w2*w_3 + w1w2*w_4 + 2*w_real*w_3*w_4)/g0;
    float s2 = (2*g1*w_real + g1*w_3 + g1*w_4 + w1w2 + 2*w_real*w_3 + 2*w_real*w_4 + w_3*w_4 + g1*g1)/g0;
    float t2 = w1w2/g0;
    float t1 = -t2*(w_3 + w_4);
    float t0 = t2*w_3*w_4;

    // Seperate integrator part
    float sf1 = s2, sf0 = s1 - s0/r0;
    float tf1 = t2, tf0 = t1 - t0/r0;
    float If = s0/r0;

    // Discretization
    unsigned int Fs;
    if (w_3 < -70) Fs = 625; else Fs = 400;
    float Ts_d = 1/((float) Fs);

    // Tustin method on seperate integrator version
    float mr = Ts_d*r0 + 2;
    float d1 = (2*sf1 + Ts_d*sf0)/mr, d0 = (Ts_d*sf0 - 2*sf1)/mr;
    float c1 = (2*tf1 + Ts_d*tf0)/mr, c0 = (Ts_d*tf0 - 2*tf1)/mr;
    float f0 = (Ts_d*r0 - 2)/mr;
    float Id = If*Ts_d;

    // Store data in return struct
    cp->cMode = 0;
    cp->Fs = Fs;
    cp->nd = 2; cp->d[0] = d0; cp->d[1] = d1;
    cp->nc = 2; cp->c[0] = c0; cp->c[1] = c1;
    cp->nf = 1; cp->f[0] = f0;
    cp->I = Id;

    return CONTROL_TUNING_SUCCESFUL;
}

/*
 * k is the pole of the speed controller
 */
unsigned char PD_cascaded_position_controller(float k,float* P, float* D) {
    if (k < 0) k = -1*k;
    *P = CASCACED_CONTROLLER_CLOSED_LOOP_POLE;
    *D = CASCACED_CONTROLLER_CLOSED_LOOP_POLE/k;
    return CONTROL_TUNING_SUCCESFUL;
}

unsigned char PID_cascaded_position_controller(float k,float* P, float* I, float* D) {
    if (k < 0) k = -1*k;
    float w_real = DOUBLE_POLE_REAL_FACTOR*CASCACED_CONTROLLER_CLOSED_LOOP_POLE;
    float w_imag = DOUBLE_POLE_IMAG_FACTOR*CASCACED_CONTROLLER_CLOSED_LOOP_POLE;
    float w1w2 = (w_real*w_real + w_imag*w_imag);
    *P = -(2*k*w_real - w1w2)/k;
    *I = w1w2;
    *D = -(2*w_real)/k;

    return CONTROL_TUNING_SUCCESFUL;
}

#endif
