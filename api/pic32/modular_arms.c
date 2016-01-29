#ifndef MODULAR_ARMS_C
#define	MODULAR_ARMS_C

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING // Removes spam
#include <plib.h>
#include <xc.h>
#include "modular_arms.h"


// Main init
void init_arm(UINT32 system_peripheral_clock) {
    UINT32 actualClock = InitI2C(system_peripheral_clock,I2C_CLOCK_FREQ);
#ifdef PRINT_DEBUG_MESSAGES
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
    {
        sprintf(buf,"Error: I2C1 clock frequency (%u) error exceeds 10%%.\n\r", (unsigned)actualClock);
        putsUART1(buf);
    }
#endif
}


// Read float
UINT8 node_read_float(UINT8 address, UINT8 read_id, float* output) {
    // Transmit the control code to read the correct data
    unsigned char sendData = read_id;
    unsigned char i2c_status = TransmitData(address,&sendData,1);
    if (i2c_status != 0) {
#ifdef PRINT_DEBUG_MESSAGES
        sprintf(buf,"Error: Failed to transmit to motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf);
#endif
        *output = 0;
        return i2c_status;
    }

    // Read the data
    unsigned char recData[4];
    i2c_status = ReadData(address,recData,4);
    if (i2c_status != 0) {
#ifdef PRINT_DEBUG_MESSAGES
        sprintf(buf,"Error: Failed to read from motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf);
#endif
        *output = 0;
    } else {
        *output = *((float*) &recData[0]);
    }

    return i2c_status;
}

// Read byte
UINT8 node_read_byte(UINT8 address, UINT8 read_id, UINT8* output){

    // Transmit the control code to read the correct data
    unsigned char sendData = read_id;
    unsigned char i2c_status = TransmitData(address,&sendData,1);
    if (i2c_status != 0) {
#ifdef PRINT_DEBUG_MESSAGES
        sprintf(buf,"Error: Failed to transmit to motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf);
#endif
        *output = 255;
        return i2c_status;
    }

    // Read the data
    i2c_status = ReadData(address,output,1);
    if (i2c_status != 0) {
#ifdef PRINT_DEBUG_MESSAGES
        sprintf(buf,"Error: Failed to read from motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf);
#endif
        *output = 255;
    }

    return i2c_status;
}

// Writes a single command
UINT8 node_write_command(UINT8 address, UINT8 command) {
    unsigned char i2c_status = TransmitData(address,&command,1);
#ifdef PRINT_DEBUG_MESSAGES
    if (i2c_status != 0) {
        sprintf(buf,"Error: Failed to transmit to motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf); }
#endif
    return i2c_status;
}

// Writes a command and then one byte of data
UINT8 node_write_byte(UINT8 address, UINT8 command, UINT8 write_data) {
    unsigned char sendData[2] = {0};
    sendData[0] = command;
    sendData[1] = write_data;

    unsigned char i2c_status = TransmitData(address,sendData,2);
#ifdef PRINT_DEBUG_MESSAGES
    if (i2c_status != 0) {
        sprintf(buf,"Error: Failed to transmit to motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf); }
#endif
    return i2c_status;

}

// Writes a float to the controller
UINT8 node_write_float(UINT8 address, UINT8 write_id, float write_data){
    unsigned char sendData[5] = {0}, i;
    sendData[0] = write_id;
    for (i = 0; i < 4; i++) { //Might exist a sexier way of doing this
        sendData[(i+1)] = *( ((unsigned char*) &write_data) + i);
    }
    unsigned char i2c_status = TransmitData(address,sendData,5);
#ifdef PRINT_DEBUG_MESSAGES
    if (i2c_status != 0) {
        sprintf(buf,"Error: Failed to transmit to motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf); }
#endif
    return i2c_status;
}

// Tune control parameters
UINT8 tune_control_params(UINT8 node_id, UINT8 c_type, float J, float w) {
    unsigned char sendData[1 + 2*sizeof(float)] = {0}, i;
    sendData[0] = I2C_TUNE_CONTROL_PARAMS;
    sendData[1] = c_type;
    for (i = 0; i < sizeof(float); i++) {
        sendData[2+i] = *( ((unsigned char*) &J) + i);
        sendData[2+sizeof(float)+i] = *( ((unsigned char*) &w) + i);
    }

#ifdef PRINT_DEBUG_MESSAGES
    for (i=0; i < 10; i++) {
        sprintf(buf,"%x ",sendData[i]);
        putsUART1(buf);
    }
    putsUART1("\n\r");
#endif


    unsigned char i2c_status = TransmitData(ID_TO_ADDRESS(node_id),sendData,10);
#ifdef PRINT_DEBUG_MESSAGES
    if (i2c_status != 0) {
        sprintf(buf,"Error: Failed to transmit tuning params to motor, error code: %d.\n\r", i2c_status);
        putsUART1(buf); }
#endif
    return i2c_status;
}

// Program control parameters
UINT8 calculate_checksum(control_params_struct* params) {
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
    return ~((UINT8) sum);
}
UINT8 program_control_params(UINT8 node_id, control_params_struct* new_params) {
    UINT8 sendData[60] = {0};

    sendData[0] = I2C_PROGRAM_NEW_CONTROL_PARAMS;
    UINT8 i, offset = 1;
    //cMode
    sendData[offset] = new_params->cMode;
    offset += 1;
    // Fs
    sendData[offset] = *( ((UINT8*)  &(new_params->Fs)) + 0);
    sendData[(offset+1)] = *( ((UINT8*)  &(new_params->Fs)) + 1);
    offset += 2;
    // d
    sendData[offset] = new_params->nd;
    offset += 1;
    for(i = 0; i < sizeof(float)*(new_params->nd);i++){
        sendData[offset] = *( ((UINT8*)  &(new_params->d)) + i);
        offset += 1;
    }
    // c
    sendData[offset] = new_params->nc;
    offset += 1;
    for(i = 0; i < sizeof(float)*(new_params->nc);i++){
        sendData[offset] = *( ((UINT8*)  &(new_params->c)) + i);
        offset += 1;
    }
    // f
    sendData[offset] = new_params->nf;
    offset += 1;
    for(i = 0; i < sizeof(float)*(new_params->nf);i++){
        sendData[offset] = *( ((UINT8*)  &(new_params->f)) + i);
        offset += 1;
    }
    // I
    for(i = 0; i < sizeof(float); i++) {
        sendData[offset] = *( ((UINT8*)  &(new_params->I)) + i);
        offset += 1;
    }

    sendData[offset] = calculate_checksum(new_params);
    offset += 1;

#ifdef PRINT_DEBUG_MESSAGES
    sprintf(buf,"Size of control parameters: %d, checksum: %d.\n\r",offset,sendData[offset-1],new_params->I);
    putsUART1(buf);
    sprintf(buf,"%x %x %x %x\n\r",sendData[offset-5],sendData[offset-4],sendData[offset-3],sendData[offset-2]);
    putsUART1(buf);
#endif
    unsigned char i2c_status = TransmitData(ID_TO_ADDRESS(node_id),sendData,offset);
#ifdef PRINT_DEBUG_MESSAGES
    if (i2c_status != 0) {
        sprintf(buf,"Error: Failed to transmit to motor when programming control params, error code: %d.\n\r", i2c_status);
        putsUART1(buf); }
#endif

    return i2c_status;
}




#endif	/* MODULAR_ARMS_C */
