#ifndef MAIN_DECLARATIONS_C
#define MAIN_DECLARATIONS_C

#include <plib.h>
#include <xc.h>
#include "main_declarations.h"

void init_adc(void) {
    // configure and enable the ADC
    CloseADC10();    // ensure the ADC is off before setting the configuration
    AD1CHS = ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN2; // configure to sample AN2
    mPORTBSetPinsAnalogIn(SKIP_SCAN_ALL);
    AD1CSSL = ~ENABLE_AN2_ANA;
    AD1CON3 = ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_16Tcy;
    AD1CON2 = ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | \
              ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF;
    AD1CON1 = ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_MANUAL | ADC_AUTO_SAMPLING_OFF;
    EnableADC10(); // Enable the ADC
    AcquireADC10();
}

UINT16 read_adc(UINT8 adc_buf) {
    ConvertADC10();
    while(!BusyADC10());
    AcquireADC10();
    return ReadADC10(adc_buf);

}

void delay_ms(UINT32 ms) {
	UINT32 start_millis = millis;
	while ((millis - start_millis) < ms);
}

#endif