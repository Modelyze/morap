#ifndef LMD18200_DRIVERS
#define	LMD18200_DRIVERS

#include "LMD18200_drivers.h"
//#include <PIC24F_plib.h>
#include <xc.h>

void init_motor(void) {
    // Enable pin (PWM input on the LMD18200)
    TRISBbits.TRISB12 = 0;
    MOTOR_DISABLE();

    // Brake pin
    TRISBbits.TRISB13 = 0;
    MOTOR_BRAKE_OFF();

    // THERMAL_FLAG pin
    TRISBbits.TRISB15 = 1;
    ANSBbits.ANSB15 = 0;

    // Configures PWM output (direction pin on the LMD18200)
    // 50% PWM still, 0% full left, 100% full right

    TRISAbits.TRISA6 = 0; // Output

    // Timer 2 for PWM
    T2CONbits.T32 = 0;      // pwm can only operate in 16-bit mode
    T2CONbits.TCKPS = 0;    // prescaler: 0 = 1:1, 1 = 1:8, 2 = 1:64, 3 = 1:256
    T2CONbits.TCS = 0;      // internal clock source
    T2CONbits.TON = 1;      // Enable
#ifdef USE_CENTER_ALIGNED_PWM
    // For center aligned the period goes into the timer period register
    PR2 = MOTOR_PWM_PERIOD;

    // OC1-pin
    OC1RS = (unsigned int)(((float)MOTOR_PWM_PERIOD)*0.75);    // Go low value
    OC1R =  (unsigned int)(((float)MOTOR_PWM_PERIOD)*0.25);  // Go high value
    OC1CON2bits.SYNCSEL = 0b01100;
    OC1CON2bits.OCTRIG = 0;
    OC1CON2bits.DCB = DCB_OFFSET;
    OC1CON1bits.OCTSEL = 0;     // Timer2 input
    OC1CON1bits.OCM = 0b111;    // Center-aligned PWM mode
#else
#ifndef USE_EDGE_ALIGNED_PWM
#warning PWM mode not defined, assuming edge aligned
#endif
    PR2 = 0xFFFF;

    // OC1-pin
    OC1RS = MOTOR_PWM_PERIOD;    // Period
    OC1R =  ((unsigned int) ((float)MOTOR_PWM_PERIOD)/2);  // Duty cycle
    OC1CON2bits.SYNCSEL = 0x1F;
    OC1CON2bits.OCTRIG = 0;
    OC1CON2bits.DCB = 0;
    OC1CON1bits.OCTSEL = 0;     // Timer2 input
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
#endif


    // Configures ADC for current sensing
    TRISAbits.TRISA0 = 1;   // Input
    ANSAbits.ANSA0 = 1;     // Analog signal
    AD1CON2bits.PVCFG = 0;  // AVdd (Vdd) positive reference voltage
    AD1CON2bits.NVCFG = 0;  // AVss (Vss) negative reference voltage
    AD1CON1bits.MODE12 = 1; // 12 bit resolution
    AD1CON1bits.FORM = 0;   // Unsigned integer
    AD1CON1bits.SSRC = 0;   // Manual sampling
    AD1CON1bits.ASAM = 0;   // Don't autosample
    AD1CON3bits.ADRC = 0;   // Internal clock as clock source

    AD1CHSbits.CH0SA = 0;   // AN0 positive
    AD1CHSbits.CH0NA = 0;   // AVss negative

    AD1CON1bits.ADON = 1;   // Turns on the module
    AD1CON1bits.SAMP = 1;   // Starts a sampling

}

void ADC_sample(void) {
    AD1CON1bits.SAMP = 0;   // Starts conversion
}

unsigned int ADC_read(void) {
    while(AD1CON1bits.DONE == 0);   // Waits for it to finish
    AD1CON1bits.SAMP = 1;   // Starts a new sampling
    return ADC1BUF0;
}

float read_current(void) {
    ADC_sample();
    return ((((float)ADC_read()) * 3.3 / 4095.0)/CURRENT_SENSE_RESISTOR ) / (0.000377);
}

float read_voltage(void) {
#ifdef USE_DCB_BITS
    float dc = (((float) (OC1RS - OC1R)) + (((float) OC1CON2bits.DCB) - DCB_OFFSET )/4.0) / ((float)MOTOR_PWM_PERIOD + 1.0);
#else
    float dc = ((float) (OC1RS - OC1R)) / (MOTOR_PWM_PERIOD + 1.0);
#endif
    return (dc - 0.5)*SUPPLY_VOLTAGE/0.5;
}

void set_motor_speed(float spd) {
    if (spd < -LIMIT_VOLTAGE) spd = -LIMIT_VOLTAGE;
    if (spd > LIMIT_VOLTAGE) spd = LIMIT_VOLTAGE;

#ifdef USE_CENTER_ALIGNED_PWM
    float dc = 0.5*spd/SUPPLY_VOLTAGE + 0.5;
    unsigned int diff = (unsigned int) (4.0 * dc * ((float) MOTOR_PWM_PERIOD + 1.0) + 0.5);
#ifdef USE_DCB_BITS
    unsigned char dcb_temp = (diff & 0b11) + DCB_OFFSET;
    OC1CON2bits.DCB = dcb_temp & 0b11;
    diff = diff + (dcb_temp & 0xfc);
#endif
    unsigned int offset = ((MOTOR_PWM_PERIOD + 1) - (diff >> 2)) / 2;
    if ( diff == 0 ) {
        OC1RS = 0;
        OC1R = (MOTOR_PWM_PERIOD + 1);
    } else {
        OC1RS = offset + (diff >> 2);
        OC1R =  offset;
    }

#else
    OC1R = ((unsigned int) ( 0.5*((float) MOTOR_PWM_PERIOD)*spd/SUPPLY_VOLTAGE + 0.5*((float)MOTOR_PWM_PERIOD) ) ) ;
#endif
}

void set_delayed_motor_speed(float spd, unsigned char task) {
#ifdef USE_CENTER_ALIGNED_PWM
    static unsigned int OC1R_NEW = (unsigned int)(((float)MOTOR_PWM_PERIOD)*0.25);
    static unsigned int OC1RS_NEW = (unsigned int)(((float)MOTOR_PWM_PERIOD)*0.75);
    static unsigned char DCB_NEW = 0;
#else
    static unsigned int OC1R_NEW = ((unsigned int) ((float)MOTOR_PWM_PERIOD)/2);
#endif
    static unsigned char new_stuff_available = 0;
    static float input_voltage = 0.0;

    if (task == 0) {
        // Calculate all the floating point values
        if (spd < -LIMIT_VOLTAGE) spd = -LIMIT_VOLTAGE;
        if (spd > LIMIT_VOLTAGE) spd = LIMIT_VOLTAGE;
        
#ifdef USE_CENTER_ALIGNED_PWM
        float dc = 0.5*spd/SUPPLY_VOLTAGE + 0.5;
        unsigned int diff = (unsigned int) (4.0 * dc * ((float) MOTOR_PWM_PERIOD + 1.0) + 0.5);
#ifdef USE_DCB_BITS
        unsigned char dcb_temp = (diff & 0b11) + DCB_OFFSET;
        DCB_NEW = dcb_temp & 0b11;
        diff = diff + (dcb_temp & 0xfc);
#endif
        unsigned int offset = ((MOTOR_PWM_PERIOD + 1) - (diff >> 2)) / 2;
        if ( diff == 0 ) {
            OC1RS_NEW = 0;
            OC1R_NEW = (MOTOR_PWM_PERIOD + 1);
        } else {
            OC1RS_NEW = offset + (diff >> 2);
            OC1R_NEW =  offset;
        }
#else
        OC1R_NEW = ((unsigned int) ( 0.5*((float) MOTOR_PWM_PERIOD)*spd/SUPPLY_VOLTAGE + 0.5*((float)MOTOR_PWM_PERIOD) ) ) ;
#endif
        new_stuff_available = 1;
        input_voltage = spd;
    } else if (new_stuff_available) {
#ifdef USE_CENTER_ALIGNED_PWM
        OC1RS = OC1RS_NEW;
        OC1R = OC1R_NEW;
#ifdef USE_DCB_BITS
        OC1CON2bits.DCB = DCB_NEW;
#endif
        new_stuff_available = 0;
#else
        OC1R = OC1R_NEW;
#endif
//#ifdef MINIMUM_VOLTAGE
//        if (input_voltage < MINIMUM_VOLTAGE && input_voltage > -MINIMUM_VOLTAGE)
//            MOTOR_DISABLE();
//        else
//            MOTOR_ENABLE();
//#endif
    }

}

void set_duty_cycle(float dc) {
    if (dc > 100.0) dc = 100;
    if (dc < 0.0) dc = 0;

#ifdef USE_CENTER_ALIGNED_PWM
    unsigned int diff = (unsigned int) (4.0 * dc/100.0 * ((float) MOTOR_PWM_PERIOD + 1.0) + 0.5);
#ifdef USE_DCB_BITS
    unsigned char dcb_temp = (diff & 0b11) + DCB_OFFSET;
    OC1CON2bits.DCB = dcb_temp & 0b11;
    diff = diff + (dcb_temp & 0xfc);
#endif
    unsigned int offset = ((MOTOR_PWM_PERIOD + 1) - (diff >> 2)) / 2;
    if ( diff == 0 ) {
        OC1RS = 0;
        OC1R = (MOTOR_PWM_PERIOD + 1);
    } else {
        OC1RS = offset + (diff >> 2);
        OC1R =  offset;
    }
#else
    OC1R = (unsigned int) ( dc/100.0 * ((float) MOTOR_PWM_PERIOD) );
#endif
}


#ifdef USE_CENTER_ALIGNED_PWM
void set_PWM_period(unsigned int rs, unsigned int r, unsigned int dcb) {
    OC1RS = rs;
    OC1R = r;
    OC1CON2bits.DCB = dcb;
}
#else
void set_PWM_period(unsigned int p) {
    if (p > MOTOR_PWM_PERIOD) p = MOTOR_PWM_PERIOD;
    OC1R = p;
}
#endif

float get_duty_cycle(void) {
#ifdef USE_CENTER_ALIGNED_PWM
    return 100.0 * ((float)(OC1RS - OC1R)/((float)MOTOR_PWM_PERIOD + 1.0) );
#else
    return 100.0 * ((float)OC1R)/((float)MOTOR_PWM_PERIOD);
#endif
}


#endif