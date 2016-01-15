/* 
 * File:   eeprom_storage.h
 * Author: vkozma
 *
 * Created on May 22, 2015
 */

#ifndef EEPROM_STORAGE_H
#define	EEPROM_STORAGE_H

#include "main_declarations.h" // Needed for structs

// eeprom storage locations for the different variables
// maximum storage number = 255 words = 510 bytes
#define EEPROM_NODE_ID_OFFSET 1             // size = 1 byte ~ 1 word
#define EEPROM_MOTOR_PARAMS_OFFSET 3        // size = 32 bytes = 16 words
#define EEPROM_POS_CONTROL_PARAMS_OFFSET 20 // size = 62 bytes = 31 words
#define EEPROM_VEL_CONTROL_PARAMS_OFFSET 52 // size = 62 bytes = 31 words
// Test val stored in the eeprom to ensure proper functionality
#define EEPROM_TEST_OFFSET 2                // size = 1 word
#define EEPROM_TEST_VAL 0xBEEF // New value for every iteration

// Prototypes

// general eeprom functions
void EepSetup(void);
void EepErase(void);
unsigned int EepRead(unsigned int index);
void EepWrite(unsigned int index, unsigned int data);

// Write eeprom functions
void EepWriteTest(void);
void EepWriteNodeID(unsigned char id);
void EepWriteMotorParams(motor_params_struct* mp);
void EepWritePosControlParams(control_params_struct* cp);
void EepWriteVelControlParams(control_params_struct* cp);

// Read eeprom functions
unsigned char EepVerifyTest(void);
unsigned char EepReadNodeID(void);
void EepReadMotorParams(motor_params_struct*);
void EepReadPosControlParams(control_params_struct*);
void EepReadVelControlParams(control_params_struct*);






#endif	/* EEPROM_STORAGE_H */

