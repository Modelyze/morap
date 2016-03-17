/*
 * File:   main.c
 * Author: Viktor Kozma
 *
 * Writes the necessary data to the eeprom
 * memory of the pic24f microcontroller
 * 
 * If write was succesful the green led will
 * be off and the red led will be on
 * 
 * In order to preserve this memory the
 * programmer configuration must be set to
 * preserve the internal eeprom. This is done
 * in MPLABX for the PICKIT3 by:
 * Run -> Set Project Configuration -> Customize ->
 * -> PICKIT3 -> Preserve EEPROM Memory
 *
 * Created on August 24, 2015, 1:25 PM
 */
#include <xc.h>

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

// ON BOARD LEDS, TODO: Remove RB5 from this, just used as an extra debug led
#define INIT_LEDS() (TRISBbits.TRISB2 = 0, TRISBbits.TRISB4 = 0)
#define LED1_ON() (LATBbits.LATB2 = 1)
#define LED1_OFF() (LATBbits.LATB2 = 0)
#define LED1_SWAP() (LATBbits.LATB2 = ~(LATBbits.LATB2))
#define LED2_ON() (LATBbits.LATB4 = 1)
#define LED2_OFF() (LATBbits.LATB4 = 0)
#define LED2_SWAP() (LATBbits.LATB4 = ~(LATBbits.LATB4))

void delay_us(unsigned int us) {

    unsigned long int count_to_val,i;
    count_to_val = ((unsigned long int) ( (float) ((unsigned long int) us)*16)/13.0);

    for(i = 0; i < count_to_val; i++);
}




// EEPROM CONSTANTS

// eeprom storage locations for the different variables
// maximum storage number = 255 words = 510 bytes
#define EEPROM_NODE_ID_OFFSET 1             // size = 1 byte ~ 1 word
#define EEPROM_MOTOR_PARAMS_OFFSET 3        // size = 32 bytes = 16 words
#define EEPROM_POS_CONTROL_PARAMS_OFFSET 20 // size = 62 bytes = 31 words
#define EEPROM_VEL_CONTROL_PARAMS_OFFSET 52 // size = 62 bytes = 31 words
// Test val stored in the eeprom to ensure proper functionality
#define EEPROM_TEST_OFFSET 2                // size = 1 word
#define EEPROM_TEST_VAL 0xBEEF // New value for every iteration

// Programmable constants

// The structs
// Control parameter struct
typedef struct tag_control_params_struct {
    unsigned char cMode;    // 0 = position control, 1 = speed control
    unsigned int Fs;        // Sampling frequency
    unsigned char nd;       // number of feedback numerator constants
    float d[4];             // Feedback numerator constants
    unsigned char nc;       // number of feedforward numerator constants
    float c[4];             // Feedforward numerator constants
    unsigned char nf;       // number of feedback/forward denominator constants
    float f[4];             // Feedback/forward denominator constants
    float I;                // Optional integrator for both feedback/forward
} control_params_struct;

// Motor parameter struct for storing motor data
typedef struct tag_motor_params_struct {
    float R,L,K,n,c,eta,imax,ppr;
} motor_params_struct;


//***********************************************************************//


// The values to program into the eeprom

// Predefined parameters for available motors
//#define DCX26L_150
#define DCX22L_238

#if defined(DCX26L_150)
unsigned char motor_id = 1;
control_params_struct posControlParams =
{
	.cMode = 0,
	.Fs = 625,
	.nd = 2, .d = {-1.5937,2.0104},
	.nc = 2, .c = {-1.5937,2.0104},
	.nf = 1, .f = {-0.9716},
	.I = 0.0000
};
control_params_struct velControlParams =
{
	.cMode = 1,
	.Fs = 400,
	.nd = 2, .d = {0.0284,0.0284},
	.nc = 2, .c = {-0.5653,0.8839},
	.nf = 1, .f = {-0.6251},
	.I = 0.2524
};
// Motor paramters (DCX26L) w/ LMD18200
motor_params_struct motorData =
{
    .R = 0.74 + 0.33, .L = 0.000129,
    .K = 0.0214, .n = 328509/2197,
    .c = 0.05, .eta = 0.75,
    .imax = 1.5, .ppr = 128
};
#elif defined(DCX22L_238)
unsigned char motor_id = 2;
control_params_struct posControlParams =
{
        .cMode = 0,
	.Fs = 625,
	.nd = 2, .d = {-1.5711,2.2623},
	.nc = 2, .c = {-1.5711,2.2623},
	.nf = 1, .f = {-0.9716},
        .I = 0
};
control_params_struct velControlParams =
{
	.cMode = 1,
	.Fs = 500,
	.nd = 2, .d = {0.0604,0.0604},
	.nc = 2, .c = {-1.0024,1.6352},
	.nf = 1, .f = {-0.5948},
	.I = 0.4904
};
// Motor paramters (DCX22L) w/ LMD18200
motor_params_struct motorData =
{
    .R = 1.83 + 0.33, .L = 0.000192,
    .K = 0.0229, .n = 300564.0/1300.0,
    .c = 0.1, .eta = 0.74,
    .imax = 1.5, .ppr = 128
};
#endif

//***********************************************************************//

// Important variable
unsigned int __attribute__ ((space(eedata))) ee_addr = 0x1234;

// Prototype functions
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

//#define EEPROM_ONLY_READ_MODE
int main(void) {
    // 8 MHz x 4 PLL from FRC
    CLKDIVbits.RCDIV = 0;
    CLKDIVbits.DOZEN = 0;

    INIT_LEDS(); // Leds init for feedback
    LED1_OFF(); LED2_OFF();


    // Delay function, is needed because programming will turn the system
    // on - off rapidly, potentially corrupting the eeprom if this program
    // restarts during it
    int i;
    for (i = 0; i < 500; i++)
    {
        if (i % 100 == 0) LED1_SWAP();
        delay_us(10000);
    }
    LED1_ON();

    EepSetup();

#ifndef EEPROM_ONLY_READ_MODE

    // Erase the eeprom of anything uneccesary
    EepErase();

    // Program in test variable
    EepWriteTest();
    // Program in node id
    EepWriteNodeID(motor_id);
    // Program in motor params
    EepWriteMotorParams(&motorData);
    // Program in default control params
    EepWritePosControlParams(&posControlParams);
    EepWriteVelControlParams(&velControlParams);

#endif


    // Verify functionality
    if (EepVerifyTest()) LED2_ON();

    // *********************************** //
    // VERIFY CORRECT WRITTEN VARIABLES
    // (in debug menu)
    unsigned char motor_id2;
    motor_params_struct motorData2;
    control_params_struct posControlParams2, velControlParams2;

    motor_id2 = EepReadNodeID();
    EepReadMotorParams(&motorData2);
    EepReadPosControlParams(&posControlParams2);
    EepReadVelControlParams(&velControlParams2);


    unsigned char so = sizeof(control_params_struct);

    LED1_OFF();
    while(1){
        // Do nothing
    }
    return 0;
}

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