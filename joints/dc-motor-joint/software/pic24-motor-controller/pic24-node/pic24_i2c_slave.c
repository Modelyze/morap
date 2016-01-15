#ifndef PIC24_I2C_SLAVE_C
#define	PIC24_I2C_SLAVE_C

#include "pic24_i2c_slave.h"
#include "LMD18200_drivers.h"
#include "control_tuner.h"
//#include <PIC24F_plib.h>
#include <xc.h>


/*
 * Communication status variables
 */
volatile unsigned char readMode = I2C_READMODE_ANGLE;

/*
 * New Control Parameter struct, used to store the new control parameters
 */
control_params_struct newControlParams =
{ .cMode = 0, .Fs = 100, .nd = 0, .d = {0}, .nc = 0, .c = {0}, .nf = 0, .f = {0}, .I = 0 };

/*
 * New floats gets stored in this variable while being built
 */
float new_floats = 0;

// Used to store data when receving controller tuning data
unsigned char type_rec;
float J_rec, w_rec;

void init_i2c(unsigned char id) {

    // BAUD RATE REGISTER
    // For Fosc = 32MHz -> Fcy = 16 MHz: 100kHz -> 157, 400kHz -> 37
    I2C1BRG = 157;

#ifndef ENABLE_I2C_CLOCK_STRETCH
    I2C1CON = 0x8000;
#else
    I2C1CON = 0x9040;
#endif

    I2C1ADD = (I2C_CONTROL_CODE << 3) | id;

    //Enable interrupt
    IPC4bits.SI2C1IP = PRIORITY_COMM; // Priority
    IFS1bits.SI2C1IF = 0; // Clear flag
    IEC1bits.SI2C1IE = 1; // Enable

    // Status variables init
    readMode = I2C_READMODE_ANGLE;
}

// Function prototypes, defined at bottom
void AssembleNewControlParams(unsigned char,unsigned char);
unsigned char ControlParamsBytes(control_params_struct*, unsigned char);

#define DA_IS_ADDRESS() (I2C1STATbits.D_A == 0)
#define DA_IS_DATA() (I2C1STATbits.D_A == 1)
#define RW_IS_WRITE() (I2C1STATbits.R_W == 0)
#define RW_IS_READ() (I2C1STATbits.R_W == 1)
void _ISR_NO_PSV _SI2C1Interrupt(void) {
    static unsigned char n_sent = 0, id = 0;

    unsigned char timmy;    // used for dummy operations
    if ( DA_IS_ADDRESS() && RW_IS_WRITE() ) {
        // Nothing here can be done, just read the address bit to
        // start recieve data
        timmy = I2C1RCV;
        n_sent = 0;
        //LED1_SWAP();
    } else if ( DA_IS_DATA() && RW_IS_WRITE() ) {
        timmy = I2C1RCV; // Read data
        if (n_sent == 0) {
            id = timmy; // First one received is the id of the transmission
        }

        // Do stuff dependant on the id
        switch(id) {
            case I2C_DISABLE_MOTOR:
                systemStatus.control_mode = CONTROL_MODE_DISABLED;
                break;
            case I2C_SET_POSITION_REFERENCE:
                systemStatus.control_mode = CONTROL_MODE_POSITION_FEEDBACK;
                if (n_sent > 0) {
                    *( ((unsigned char*) &new_floats) + (n_sent - 1)) = timmy;
                    if ( n_sent == sizeof(float))
                        pos_ref = new_floats;
                }
                break;
            case I2C_SET_SPEED_REFERENCE:
                systemStatus.control_mode = CONTROL_MODE_SPEED_FEEDBACK;
                if (n_sent > 0) {
                    *( ((unsigned char*) &new_floats) + (n_sent - 1)) = timmy;
                    if ( n_sent == sizeof(float))
                        vel_ref = new_floats;
                }
                break;
            case I2C_SET_DRIVE_VOLTAGE:
                systemStatus.control_mode = CONTROL_MODE_VOLTAGE;
                if (n_sent > 0) {
                    *( ((unsigned char*) &new_floats) + (n_sent - 1)) = timmy;
                    if ( n_sent == sizeof(float)) {
                        drive_voltage = new_floats;
                    }
                }
                break;
            case I2C_SET_TRAJECTORY:
                // TODO: implement trajectory planner
                break;
            case I2C_CALIBRATE_ENCODER_ZERO:
                encoder_value = 0;
                systemStatus.enc_calib = ENCODER_IS_CALIBRATED;
                LED2_OFF();
                break;
            case I2C_CALIBRATE_ENCODER_PROVIDED:
                if (n_sent > 0) {
                    *( ((unsigned char*) &new_floats) + (n_sent - 1)) = timmy;
                    if (n_sent == sizeof(float)) {
                        encoder_value = new_floats*motorData.n/ENCODER_TO_RAD(motorData.ppr);
                    }
                }
                break;
            case I2C_SET_CALIBRATE_STATUS_UNKNOWN:
                systemStatus.enc_calib = ENCODER_CALIBRATION_NEEDED;
                systemStatus.control_mode = CONTROL_MODE_DISABLED;
                LED2_OFF();
                break;
            case I2C_PROGRAM_NEW_CONTROL_PARAMS:
                if (n_sent > 0) {
                    AssembleNewControlParams( (n_sent - 1), timmy );
                }
                break;

            case I2C_USE_DEFAULT_CONTROL_PARAMS:
                if (n_sent == 0) {
                    systemStatus.new_control_available = NEW_CONTROLLER_DEFAULT;
                    systemStatus.control_prog_status = CONTROL_PROG_STATUS_NOTHING;
                }
                break;
            case I2C_TUNE_CONTROL_PARAMS:
                // receive order: type (byte) - J (float) - w (float)
                if (n_sent < 2)
                    type_rec = timmy;
                else if (n_sent < (2 + sizeof(float)))
                    *( ((unsigned char*) &J_rec) + (n_sent - 2)) = timmy;
                else if (n_sent < (2 + 2*sizeof(float)))
                    *( ((unsigned char*) &w_rec) + (n_sent - (2+sizeof(float))) ) = timmy;
                if (n_sent == (1+2*sizeof(float))) {
                    if (tune_control_parameters(type_rec,J_rec,w_rec) == CONTROL_TUNING_SUCCESFUL) {
                        systemStatus.control_prog_status = CONTROL_PROG_STATUS_SUCCESS;
                        systemStatus.new_control_available = NEW_CONTROLLER_AVAILABLE;
                    } else {
                        systemStatus.control_prog_status = CONTROL_PROG_STATUS_FAILED;
                    }
                }
                break;
            case I2C_READMODE_ANGLE:
                readMode = I2C_READMODE_ANGLE;
                break;
            case I2C_READMODE_POS_REFERENCE:
                readMode = I2C_READMODE_POS_REFERENCE;
                break;
            case I2C_READMODE_VEL_REFERENCE:
                readMode = I2C_READMODE_VEL_REFERENCE;
                break;
            case I2C_READMODE_STATUS:
                readMode = I2C_READMODE_STATUS;
                break;
            case I2C_READMODE_CONTROL_PROG_STATUS:
                readMode = I2C_READMODE_CONTROL_PROG_STATUS;
                break;
            case I2C_READMODE_VOLTAGE:
                readMode = I2C_READMODE_VOLTAGE;
                break;
            case I2C_READMODE_CURRENT:
                readMode = I2C_READMODE_CURRENT;
                break;
            case I2C_READMODE_WHO_AM_I:
                readMode = I2C_READMODE_WHO_AM_I;
                break;
            case I2C_READMODE_POS_CONTROL_PARAMS:
                readMode = I2C_READMODE_POS_CONTROL_PARAMS;
                break;
            case I2C_READMODE_VEL_CONTROL_PARAMS:
                readMode = I2C_READMODE_VEL_CONTROL_PARAMS;
                break;
            case I2C_TURN_ON_LED:
                //LED1_ON();
                break;
            case I2C_TURN_OFF_LED:
                //LED1_OFF();
                break;
            case I2C_SWAP_LED:
                //LED1_SWAP();
                break;
        }

        n_sent++;
    } else if (RW_IS_READ() ) {
        if ( DA_IS_ADDRESS() ) {
            n_sent = 0;
            //LED1_SWAP();
        }
        timmy = I2C1RCV; // clears some buffer

        switch(readMode) {
            case I2C_READMODE_ANGLE:
                if (n_sent == 0)
                    new_floats = ENCODER_TO_RAD(motorData.ppr)*encoder_value/motorData.n;
                I2C1TRN = *( ((unsigned char*) &new_floats) + n_sent);
                break;
            case I2C_READMODE_POS_REFERENCE:
                if (n_sent == 0)
                    new_floats = pos_ref;
                I2C1TRN = *( ((unsigned char*) &new_floats) + n_sent);
                break;
            case I2C_READMODE_VEL_REFERENCE:
                if (n_sent == 0)
                    new_floats = vel_ref;
                I2C1TRN = *( ((unsigned char*) &new_floats) + n_sent);
                break;
            case I2C_READMODE_STATUS:
                if (systemStatus.enc_calib == ENCODER_CALIBRATION_NEEDED) {
                    I2C1TRN = 3;
                } else if (systemStatus.control_mode == CONTROL_MODE_DISABLED) {
                    I2C1TRN = 2;
                } else {
                    I2C1TRN = 1;
                }
                break;
            case I2C_READMODE_CONTROL_PROG_STATUS:
                I2C1TRN = systemStatus.control_prog_status;
                break;
            case I2C_READMODE_VOLTAGE:
                if (n_sent == 0)
                    new_floats = read_voltage();
                I2C1TRN = *( ((unsigned char*) &new_floats) + n_sent);
                break;
            case I2C_READMODE_CURRENT:
                if (n_sent == 0)
                    new_floats = read_current();
                I2C1TRN = *( ((unsigned char*) &new_floats) + n_sent);
                break;
            case I2C_READMODE_WHO_AM_I:
                timmy = I2C1ADD;
                I2C1TRN = timmy;
                break;
            case I2C_READMODE_POS_CONTROL_PARAMS:
                I2C1TRN = ControlParamsBytes(&posControlParams,n_sent);
                break;
            case I2C_READMODE_VEL_CONTROL_PARAMS:
                I2C1TRN = ControlParamsBytes(&velControlParams,n_sent);
                break;
            default: // shouldn't really happen, but if....
                I2C1TRN = 0;
                readMode = I2C_READMODE_ANGLE;
        } // End of switch(ReadMode)


        n_sent++; 
    } // end of else if(RW_IS_READ)

#ifdef LED1_MODE_I2C
    if (led1_timer_count < 0) {
        led1_timer_count = 0;
        LED1_ON();
    }
#endif

#ifdef ENABLE_I2C_CLOCK_STRETCH
    I2C1CONbits.SCLREL = 1; // Release clock
#endif
    IFS1bits.SI2C1IF = 0; // Clear flag
    
}


// Functions

// Checksum calculation for a parameter struct
unsigned char calculate_checksum(control_params_struct* params) {
    unsigned int sum = 0;
    unsigned char i;
    sum += params -> cMode;
    sum += *( ((unsigned char*) &(params->Fs)) + 0 );
    sum += *( ((unsigned char*) &(params->Fs)) + 1 );
    sum += params -> nd;
    for(i = 0; i < (params->nd)*sizeof(float);i++)
        sum += *( ((unsigned char*) &(params->d)) + i);
    sum += params -> nc;
    for(i = 0; i < (params->nc)*sizeof(float);i++)
        sum += *( ((unsigned char*) &(params->c)) + i);
    sum += params -> nf;
    for(i = 0; i < (params->nf)*sizeof(float);i++)
        sum += *( ((unsigned char*) &(params->f)) + i);
    for(i = 0; i < sizeof(float); i++)
        sum += *( ((unsigned char*) &(params->I)) + i);
    while (sum >> 8)
        sum = (sum >> 8) + (sum & 0xFF);
    return ~((unsigned char) sum);
}
// Called when assembling new control parameters
void AssembleNewControlParams(unsigned char n,unsigned char data) {
    // Has the following structure
    // cMode - Fs - nd - d[0:(nd-1)] - nc - c[0:(nc-1)] - nf - f[0:(nc-1)] - checksum

    // 0 = Fs, 1 = nd, 2 = d[], 3 = nc, 4 = c[], 5 = nf, 6 = f[], 7 = CRC
    static unsigned char write_level = 0, written_bytes = 0;
    if (n == 0) {
        write_level = 0;
        written_bytes = 0;
    }
    unsigned char chksm;
    switch(write_level) {
        case 0:
            newControlParams.cMode = data;
            write_level = 1;
            written_bytes = 0;
            break;
        case 1: // Fs
            *( ((unsigned char*) &newControlParams.Fs) + written_bytes) = data;
            written_bytes++;
            if (written_bytes > 1) {
                write_level = 2;
                newControlParams.nd = 0;
                newControlParams.nc = 0;
                newControlParams.nf = 0;
            }
            break;
        case 2: // nd
            newControlParams.nd = data;
            write_level = (data > 0) ? 3 : 4;
            if (data > 4) {
                systemStatus.control_prog_status = CONTROL_PROG_STATUS_FAILED;   // FAIL
                write_level = 100;
            }
            written_bytes = 0;
            break;
        case 3: // d[]
            *( ((unsigned char*) &newControlParams.d) + written_bytes) = data;
            written_bytes++;
            if ( written_bytes >= newControlParams.nd*sizeof(float) ) {
                write_level = 4;
            }
            break;
        case 4: // nc
            newControlParams.nc = data;
            write_level = (data > 0) ? 5 : 6;
            if (data > 4) {
                systemStatus.control_prog_status = CONTROL_PROG_STATUS_FAILED;   // FAIL
                write_level = 100;
            }
            written_bytes = 0;
            break;
        case 5: // c[]
            *( ((unsigned char*) &newControlParams.c) + written_bytes) = data;
            written_bytes++;
            if ( written_bytes >= newControlParams.nc*sizeof(float) ) {
                write_level = 6;
            }
            break;
        case 6: // nf
            newControlParams.nf = data;
            write_level = (data > 0) ? 7 : 8;
            if (data > 4) {
                systemStatus.control_prog_status = CONTROL_PROG_STATUS_FAILED;   // FAIL
                write_level = 100;
            }
            written_bytes = 0;
            break;
        case 7: // f[]
            *( ((unsigned char*) &newControlParams.f) + written_bytes) = data;
            written_bytes++;
            if ( written_bytes >= newControlParams.nf*sizeof(float) ) {
                write_level = 8;
                written_bytes = 0;
            }
            break;
        case 8: // I
            *( ((unsigned char*) &newControlParams.I) + written_bytes) = data;
            written_bytes++;
            if (written_bytes >= sizeof(float)) {
                write_level = 9;
            }
            break;
        case 9: // Checksum
            chksm = calculate_checksum(&newControlParams);
            if (data == chksm) {
                systemStatus.control_prog_status = CONTROL_PROG_STATUS_SUCCESS;   // SUCCESS
                systemStatus.new_control_available = NEW_CONTROLLER_AVAILABLE;
            } else {
                systemStatus.control_prog_status = CONTROL_PROG_STATUS_FAILED;   // FAIL
            }            
            write_level = 100; // Do nothing unless started over
            break;
    }

}
// Outputs single bytes of the control parameters
unsigned char ControlParamsBytes(control_params_struct* cp, unsigned char n) {
    const unsigned char OFFSET_FS = 1;
    const unsigned char OFFSET_D = 4;
    unsigned char OFFSET_C = (OFFSET_D + sizeof(float)*cp->nd + 1);
    unsigned char OFFSET_F = (OFFSET_C + sizeof(float)*cp->nc + 1);
    unsigned char OFFSET_I = (OFFSET_F + sizeof(float)*cp->nf);

    unsigned char toReturn = 0;
    if (n < OFFSET_FS) { // cMode
        toReturn = cp->cMode;
    } else if ( n < (OFFSET_D - 1) ) { // Fs
        toReturn = *( ((unsigned char*) &(cp->Fs)) + (n-OFFSET_FS));
    } else if (n < OFFSET_D) { // nd
        toReturn = cp->nd;
    } else if (n < (OFFSET_C - 1) ) { // d[0:nd-1]
        toReturn = *( ((unsigned char*) &(cp->d)) + (n-OFFSET_D));
    } else if ( n < (OFFSET_C) ) { // nc
        toReturn = cp->nc;
    } else if ( n < (OFFSET_F - 1) ) { // c[0:nc-1]
        toReturn = *( ((unsigned char*) &(cp->c)) + (n-OFFSET_C));
    } else if ( n < (OFFSET_F) ) { // nf
        toReturn = cp->nf;
    } else if ( n < (OFFSET_I) ) { // f
        toReturn = *( ((unsigned char*) &(cp->f)) + (n-OFFSET_F));
    } else if ( n < (OFFSET_I + sizeof(float))) { // I
        toReturn = *( ((unsigned char*) &(cp->I)) + (n-OFFSET_I));
    } else if ( n < (OFFSET_I + sizeof(float) + 1)) { // Checksum
        toReturn = calculate_checksum(cp);
    }
    return toReturn;
}

#endif