//#include <PIC24F_plib.h>
#include <xc.h>
#include "eeprom_storage.h"

// Important variable
unsigned int __attribute__ ((space(eedata))) ee_addr = 0x1234;

void EepSetup(){
    //Disable Interrupts For 5 instructions
    asm volatile("disi #5");
    //Issue Unlock Sequence
    asm volatile("mov #0x55, W0 \n"
    "mov W0, NVMKEY \n"
    "mov #0xAA, W1 \n"
    "mov W1, NVMKEY \n");
}

void EepErase(void) {
    // Erases all the eeprom memory
    NVMCON = 0x4050;            // Set up NVMCON to bulk erase the data EEPROM
    asm volatile ("disi #5");   // Disable Interrupts For 5 Instructions
    __builtin_write_NVM();      // Issue Unlock Sequence and Start Erase Cycle
    while(_WR);
}

void EepWrite(unsigned int index, unsigned int data){
    unsigned int offset;
    NVMCON = 0x4004;    // Set up NVMCON to erase one word of data EEPROM
    TBLPAG = __builtin_tblpage(&ee_addr);    // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&ee_addr);  // Initizlize lower word of address
    offset += index * sizeof(int);
    __builtin_tblwtl(offset, data);
    asm volatile ("disi #5");   // Disable Interrupts For 5 Instructions
    __builtin_write_NVM();      // Issue Unlock Sequence and Start Erase Cycle
    while(_WR);
}

unsigned int EepRead(unsigned int index){
    unsigned int offset;

    TBLPAG = __builtin_tblpage(&ee_addr);    // Initialize EE Data page pointer
    offset = __builtin_tbloffset(&ee_addr);  // Initizlize lower word of address
    offset += index * sizeof(int);
    return __builtin_tblrdl(offset);    // read EEPROM data
}

// Write functions
void EepWriteTest(void) {
    EepWrite(EEPROM_TEST_OFFSET,EEPROM_TEST_VAL);
}

void EepWriteNodeID(unsigned char id) {
    EepWrite(EEPROM_NODE_ID_OFFSET,id);
}

void EepWriteMotorParams(motor_params_struct* mp) {
    unsigned int i;
    for(i = 0;i < (sizeof(motor_params_struct)/2);i++) {
        EepWrite((EEPROM_MOTOR_PARAMS_OFFSET + i),*(((unsigned int*) mp) + i) );
    }
}

void EepWritePosControlParams(control_params_struct* cp) {
    unsigned int i;
    for(i = 0;i < (sizeof(control_params_struct)/2);i++) {
        EepWrite((EEPROM_POS_CONTROL_PARAMS_OFFSET + i),*(((unsigned int*) cp) + i) );
    }
}

void EepWriteVelControlParams(control_params_struct* cp) {
    unsigned int i;
    for(i = 0;i < (sizeof(control_params_struct)/2);i++) {
        EepWrite((EEPROM_VEL_CONTROL_PARAMS_OFFSET + i),*(((unsigned int*) cp) + i) );
    }
}

// Read functions
unsigned char EepVerifyTest(void) {
    if (EepRead(EEPROM_TEST_OFFSET) == EEPROM_TEST_VAL)
        return 1;
    return 0;
}

unsigned char EepReadNodeID(void) {
    return EepRead(EEPROM_NODE_ID_OFFSET);
}

void EepReadMotorParams(motor_params_struct* mp) {
    unsigned int i;
    for (i = 0; i < (sizeof(motor_params_struct)/2); i++) {
        *(((unsigned int*) mp) + i) = EepRead((EEPROM_MOTOR_PARAMS_OFFSET + i));
    }
}

void EepReadPosControlParams(control_params_struct* cp) {
    unsigned int i;
    for (i = 0; i < (sizeof(control_params_struct)/2); i++) {
        *(((unsigned int*) cp) + i) = EepRead((EEPROM_POS_CONTROL_PARAMS_OFFSET + i));
    }
}

void EepReadVelControlParams(control_params_struct* cp) {
    unsigned int i;
    for (i = 0; i < (sizeof(control_params_struct)/2); i++) {
        *(((unsigned int*) cp) + i) = EepRead((EEPROM_VEL_CONTROL_PARAMS_OFFSET + i));
    }
}
