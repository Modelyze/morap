#ifndef LOGGING_ROUTINES_C
#define LOGGING_ROUTINES_C

#include <plib.h>
#include <xc.h>
#include "main_declarations.h"
#include "modular_arms.h"
#include "logging_routines.h"
 
 // Logging functions
 
 
 
void TransientLogging(UINT8 node_id, float log_voltage) {
    // Logging parameters
	UINT8 i2c_status;
    float  curr_ang, curr_voltage, curry_time = 0.0, sent_voltage = 0.0, log_time = 1.0;
    UINT16 log_Ts = 10;
	UINT32 start_millis, start_log_millis;
	
    disable_motor(node_id);


    sprintf(buf,"\nlog voltage: %0.1f V, log time: %0.1f sec, Ts: %d ms\n\n\r", log_voltage, log_time, log_Ts);
    putsUART1(buf);
    putsUART1("| Time | motor angle | motor voltage |\n\n\r");

    delay_ms(100);
    putsUART1("starting logging:\n\r");

    i2c_status = calibrate_encoder_zero(node_id);
	if (i2c_status != I2C_STATUS_SUCCESFUL) {
		sprintf(buf,"Motor with id %d not found on the bus, quitting\n\r",node_id);
		putsUART1(buf);
		return;
	}
    start_millis = millis;
    //set_drive_voltage(1,log_voltage);

    unsigned char log_stage = 0;
    while( curry_time < (2.0 + log_time) ) {
		
		start_log_millis = millis;
        curry_time = ((float)(start_log_millis - start_millis))/1000.0;


        // Waits one second until activating motor
        if ( (log_stage == 0) && (curry_time > 1.0) ){
            i2c_status = set_voltage(node_id,log_voltage);
            if (i2c_status == I2C_STATUS_SUCCESFUL) {
                log_stage = 1;
                sent_voltage = log_voltage;
            }
        }

        // After log_time set voltage to zero and continue recording
        if ( (log_stage == 1) && (curry_time > (log_time + 1.0)) ) {
            i2c_status = set_voltage(node_id,0.0);
            if (i2c_status == I2C_STATUS_SUCCESFUL) {
                log_stage = 2;
                sent_voltage = 0.0;
            }
        }


        i2c_status = get_angle(node_id,&curr_ang);
		i2c_status |= get_voltage(node_id,&curr_voltage);
        if (i2c_status == I2C_STATUS_SUCCESFUL) {
            sprintf(buf,"%f, %f, %f\n\r", curry_time, curr_ang, curr_voltage);
            putsUART1(buf);
        }

        // Wait until Ts milliseconds has passed since start
        while( millis - start_log_millis < log_Ts);

    }
    putsUART1("logging completed!\n\r");
    set_calibration_status_unknown(node_id);
}

// Steps through a lot of random voltages
void RandomLogging(UINT8 node_id, float max_voltage) {
	UINT8 i2c_status;
    float  curr_ang, curry_time = 0.0, swap_time;
    UINT16 log_time = 2, log_Ts = 10, random_store = 0;
    float curr_voltage, sent_voltage;
	UINT32 start_millis, start_log_millis;
	
    i2c_status = disable_motor(node_id);
	if (i2c_status != I2C_STATUS_SUCCESFUL) {
		sprintf(buf,"Motor with id %d not found on the bus, quitting\n\r",node_id);
		putsUART1(buf);
		return;
	}
	
    sprintf(buf,"\nlog voltages: random, log time: %d sec, Ts: %d ms\n\n\r", log_time, log_Ts);
    putsUART1(buf);
    putsUART1("| Time | motor angle | motor voltage |\n\n\r");
    delay_ms(100);
    putsUART1("starting logging:\n\r");


    swap_time = ((float) log_time);

    calibrate_encoder_zero(node_id);
	set_drive_voltage(node_id,0.0);
    sent_voltage = 0.0;
    start_millis = millis;

    while(curry_time < 30.0) {
        start_log_millis = millis;
        curry_time = ((float)(start_log_millis - start_millis))/1000.0;

        i2c_status = get_angle(node_id,&curr_ang);
        i2c_status |= get_voltage(node_id,&curr_voltage);
        if (i2c_status == I2C_STATUS_SUCCESFUL) {
            sprintf(buf,"%f, %f, %f\n\r", curry_time, curr_ang, curr_voltage);
            putsUART1(buf);
        }

        // To determine next random voltage
        // This should be random enough
        //random_store = (random_store << 1) | (ReadTimer45() & 1);
        random_store = (random_store << 1) | (read_adc(0) & 1);

        if (curry_time > swap_time) {
            sent_voltage = max_voltage*((float)random_store)/((float)0xFFFFFFFF);
            i2c_status = set_voltage(node_id,sent_voltage);
            if (i2c_status == I2C_STATUS_SUCCESFUL) {                
                swap_time = swap_time + ((float) log_time);
            }
        }

        // Wait until Ts milliseconds has passed since start
        while( millis - start_log_millis < log_Ts);
    }
    putsUART1("logging completed!\n\r");
    set_calibration_status_unknown(node_id);

}

// Control test
// Feedback control test
void ControlLogging(UINT8 node_id) {
    // Logging parameters
	UINT8 i2c_status;
    float  curr_ang, curry_time = 0.0, curr_voltage = 0.0, log_time = 2.0;
	UINT32 start_millis, start_log_millis, log_Ts = 10;
    float reference = 1.5708;

//#define PROGRAM_NEW_PARAMS
#ifdef PROGRAM_NEW_PARAMS
    control_params_struct newControlParams =
    {
            .Fs = 625,
            .nd = 2, .d = {-0.6327,0.8222},
            .nc = 2, .c = {-0.6327,0.8222},
            .nf = 1, .f = {-0.9810}
    };
    i2c_status = program_control_params(node_id,&newControlParams);
    UINT8 prog_status;
    if (i2c_status == I2C_STATUS_SUCCESFUL) {
        i2c_status = read_control_prog_status(1, &prog_status);
        if(i2c_status == I2C_STATUS_SUCCESFUL && prog_status == MOTOR_CONTROL_PROGRAMMING_STATUS_SUCCESS) {
            putsUART1("Control programming succesful!\n\r");
        } else {
            putsUART1("Control programming failed, quitting...\n\r");
            return;
        }
    }
#else
    //set_default_control_params(node_id);
#endif

    sprintf(buf,"\nreference: %0.2f rad, log time: %0.1f sec, Ts: %d ms\n\n\r", reference, log_time, log_Ts);
    putsUART1(buf);
    putsUART1("| Time | motor angle | motor voltage |\n\n\r");
    delay_ms(100);
    putsUART1("starting logging:\n\r");

    i2c_status = calibrate_encoder_zero(node_id);
	if (i2c_status != I2C_STATUS_SUCCESFUL) {
		sprintf(buf,"Motor with id %d not found on the bus, quitting\n\r",node_id);
		putsUART1(buf);
		return;
	}
	set_angle(node_id,0.0);
    start_millis = millis;
	
    unsigned char log_stage = 0;
    while(curry_time < (1.0 + 2.0*log_time)) {
        start_log_millis = millis;
        curry_time = ((float)(start_log_millis - start_millis))/1000.0;
        
        if ( (log_stage == 0) && (curry_time > 1.0) ){
            i2c_status = set_angle(node_id,reference);
            if (i2c_status == I2C_STATUS_SUCCESFUL) {
                log_stage = 1;
            }
        }
        if ( (log_stage == 1) && curry_time > (1.0 + log_time)) {
            i2c_status = set_angle(node_id,0.0);
            if (i2c_status == I2C_STATUS_SUCCESFUL) {
                log_stage = 2;
            }
        }

        i2c_status = get_angle(node_id,&curr_ang);
        i2c_status |= get_voltage(node_id,&curr_voltage);
        if (i2c_status == I2C_STATUS_SUCCESFUL) {
            sprintf(buf,"%f, %f, %f\n\r", curry_time, curr_ang, curr_voltage);
            putsUART1(buf);
        }


        // Wait until Ts milliseconds has passed since start
        while( millis - start_log_millis < log_Ts);
    }
    putsUART1("logging completed!\n\r");
    set_calibration_status_unknown(node_id);
}


 #endif