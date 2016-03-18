/*
 * File:   main.c
 * Author: Viktor Kozma
 *
 * Created on January 18, 2016, 2:13 PM
 *
 * Does inverted pendulum control
 *
 * Electrical connections (uC loc(uno32 pin)):
 * On board uno32 leds on PORTG6 and PORTF0
 * RED LED ON PORTD1(5)
 * YELLOW LED ON PORTD2(6)
 * BUTTON 1 on PORTD8(2)
 * BUTTON 2 on PORTD9(7)
 *
 * 
 */

#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <xc.h>
#include <math.h>
#include "main_declarations.h"
#include "../../api/pic32/modular_arms.h" //path to modular_arms.h
#include "mpu9150.h" 


// Configs
#pragma config FPLLMUL = MUL_20, FPLLIDIV = DIV_2, FPLLODIV = DIV_1, FWDTEN = OFF
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1

// Global vars
unsigned char buf[256]; // OUTPUT BUFFER FOR UART
imu_store_struct imu_data; // Store measurements
imu_raw_store_struct imu_raw_data;
BOOL imu_is_init = FALSE;

int main(void) {
    // Configure the device for maximum performance but do not change the PBDIV
    // Given the options, this function will change the flash wait states and
    // enable prefetch cache but will not change the PBDIV. The PBDIV value
    // is already set via the pragma FPBDIV option above..
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    // SETS UP UART, calculating the baud rates are done as follows:
    // BAUD = FPG / (16* (U1BRG + 1))  ==>
    // U1BRG = FPG / (16*BAUD) - 1
    // Where FPG = F_CPU / FPBDIV ( 80 MHz )
    // So 129 for 38400, 42 for 115200
    // found on computer at /dev/ttyUSB0
    OpenUART1(UART_EN,UART_TX_ENABLE | UART_RX_ENABLE,42);

    // SETUP TIMER 1 INTERUPPT!
    // F_tick = (80e6 / Prescaler) / period
    //Open Timer1, 64 Prescaler -> Fclock = 1250000 Hz. With period = 1250 -> tick = 1000 Hz
    UINT16 T1_period;
    if (FS > 20) {
        T1_period = (UINT16) ( (GetPeripheralClock()/64L)/((UINT32) FS) );
        OpenTimer1(T1_ON | T1_PS_1_64 | T1_SOURCE_INT, T1_period);
    } else {
        T1_period = (UINT16) ( (GetPeripheralClock()/256L)/((UINT32) FS) );
        OpenTimer1(T1_ON | T1_PS_1_256 | T1_SOURCE_INT, T1_period);
    }
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);


    // Sets up internal 32-bit timer for delay functions
    OpenTimer45(T45_ON | T45_PS_1_256, 0xFFFFFFFF);

    // init adc
    init_adc();

    // Button and LEDS
    INIT_LEDS();
    LED4_OFF(); LED5_OFF();
    INIT_SHIELD_LEDS();
    INIT_PULSE_TRIGGER();
    RED_LED_OFF(); YELLOW_LED_OFF();
    INIT_BUTS();
    INIT_TS_PIN();

    // INIT the arm
    init_arm(GetPeripheralClock());

    // Waits a while until we start to initialize the IMU
    // to improve stability
    INT16 i;
    for(i = 0; i < 25; i++){
        delay_ms(100);
        LED5_SWAP();
        YELLOW_LED_SWAP();
    }
    YELLOW_LED_ON();
    LED5_OFF();


    // Init gyro
    putsUART1("INITIATES IMU\n\r");
    PULSE_TRIGGER();
    UINT8 ig = init_mpu9150();
    if (ig != I2C_STATUS_SUCCESFUL) {
        putsUART1("------------ IMU-INIT FAILED! ------------\n\r");
        imu_is_init = FALSE;
    } else {
        imu_is_init = TRUE;
    }
    YELLOW_LED_OFF();

    set_calibration_status_unknown(NODE_ID);
    
    // Tune speed control parameters (NOTE: make sure it can actually reach the desired pole)
    UINT8 i2c_status = tune_control_params(NODE_ID,TUNING_PI_SPEED_CONTROLLER,0.1,-50);
    if (i2c_status == I2C_STATUS_SUCCESFUL) {
        UINT8 prog_status;
        i2c_status = get_control_prog_status(NODE_ID, &prog_status);
        if(i2c_status == I2C_STATUS_SUCCESFUL && prog_status == MOTOR_CONTROL_PROGRAMMING_STATUS_SUCCESS) {
            sprintf(buf,"Controller programming on node %d successful\n\r",NODE_ID);
            putsUART1(buf);
        } else {
            sprintf(buf,"Controller programming on node %d FAILED!!!!!\n\r",NODE_ID);
            putsUART1(buf);
        }
    } else {
        sprintf(buf,"Motor with id %d not found on the bus\n\r",NODE_ID);
        putsUART1(buf);
    }
    // says hello
    sprintf(buf,"Systems initiated (T1 period = %d)\n\r",T1_period);
    putsUART1(buf);

    // test reads imu config register
    delay_ms(100);
    UINT8 read_data[2];
    read_data[0] = MPU9150_GYRO_CONFIG;
    TransmitData(MPU9150_I2C_ADDRESS,read_data,1);
    if (ReadData(MPU9150_I2C_ADDRESS,read_data,2) == I2C_STATUS_SUCCESFUL) {
        sprintf(buf,"gyro: 0x%x\n\raccel: 0x%x\n\r",read_data[0],read_data[1]);
        putsUART1(buf);
    }

    // Enables interrupts
    mT1ClearIntFlag();
    INTEnableSystemMultiVectoredInt();


    while(1){
        // main loop
    }
}


// Function prototypes (implemented at bottom)
void scanI2Cbus(void); // scans bus and prints found addresses
void potControl(void);

// Different modes
// -------------------------------------------
//#define DISP_RAW_MEAN_DATA
//#define DISP_SENSOR_DATA
//#define DISP_FAST_SENSOR_DATA
//#define DISP_MEASUREMENT_ANGLE
//#define DISP_KALMAN_FILTER
#define FEEDBACK_CONTROL
//#define POT_CONTROL
// -------------------------------------------

// Timer 1 interrupt routine
void __ISR(_TIMER_1_VECTOR, IPL2AUTO) _Timer1Handler(void) {
    mT1ClearIntFlag();
    TS_PIN_HIGH();

    static UINT32 cnt = 0;
    static UINT8 kalman_reset = 0;
    cnt++;
    //Read imu_data
    LED5_ON();
    UINT8 imu_i2c_status = read_imu_data(&imu_data), mtr_i2c_status;
    LED5_OFF();

#ifdef FEEDBACK_CONTROL
    
    static float th2, th1;
    if (imu_i2c_status == I2C_STATUS_SUCCESFUL && imu_is_init) {
        LED5_ON();
        mtr_i2c_status = get_angle(NODE_ID,&th1);
        LED5_OFF();
        if (mtr_i2c_status != I2C_STATUS_SUCCESFUL) RED_LED_ON();
        else RED_LED_OFF();

        if (kalman_reset == 1) putsUART1("Resetting kalman filter!\n\r");
        th2 = kalman_filtering(&imu_data,&kalman_reset);
        //th2 = complementary_filter(&imu_data);

        control_logic(th1,th2,&imu_data);        
    } else {
        kalman_reset = 1;
        disable_control();
        RED_LED_ON();
        imu_is_init = FALSE;
    }
    // MANUAL OVERRIDE
    if (GET_BUT1() || GET_BUT2()) {
        // disable motor
        disable_control();
        YELLOW_LED_OFF();
    }
#endif
#ifdef DISP_KALMAN_FILTER
    static float max_rate = -1000, min_rate = 1000;
    if (imu_i2c_status == I2C_STATUS_SUCCESFUL) {
#ifndef FEEDBACK_CONTROL
        if (kalman_reset == 1) putsUART1("Resetting kalman filter!\n\r");
        float th2 = kalman_filtering(&imu_data,&kalman_reset);
#endif
        float rate = get_control_signal(&imu_data);
        if(rate > max_rate) max_rate = rate;
        if(rate < min_rate) min_rate = rate;

        if(cnt % (FS/4) == 0) {
            //float rate = get_control_signal(&imu_data);
            sprintf(buf,"a = %0.3f deg (abs acc = %0.2fg, rate = %0.2f deg/s)\n\r",\
                    RAD_TO_DEG(th2),get_abs_plane_acc(&imu_data),RAD_TO_DEG(rate) );
            putsUART1(buf);
            sprintf(buf," (rates = %0.2f : %0.2f)\n\r",\
                    RAD_TO_DEG(min_rate),RAD_TO_DEG(max_rate) );
            putsUART1(buf);
            max_rate = -1000; min_rate = 1000;
        }
        RED_LED_OFF();
    } else {
        kalman_reset = 1;
        RED_LED_ON();
    }
#endif
#ifdef DISP_MEASUREMENT_ANGLE
    if (imu_i2c_status == I2C_STATUS_SUCCESFUL) {
        if(cnt % (FS/4) == 0) {
            sprintf(buf,"a = %0.3f deg (abs acc = %0.2fg)\n\r",RAD_TO_DEG(get_measurement(&imu_data)),get_abs_acc(&imu_data));
            putsUART1(buf);
        }
    }
#endif
#ifdef DISP_SENSOR_DATA
    // Disp data in terminal
    if(cnt % (FS/2) == 0) {
        if (imu_i2c_status == I2C_STATUS_SUCCESFUL) {
            sprintf(buf,"aX = %0.6f g, aY = %0.6f g, aZ = %0.6f g\n\r",imu_data.accX,imu_data.accY,imu_data.accZ);
            putsUART1(buf);
            sprintf(buf,"gX = %0.4f dg/s, gY = %0.4f dg/s, gZ = %0.4f dg/s\n\r",\
                    RAD_TO_DEG(imu_data.gyroX),RAD_TO_DEG(imu_data.gyroY),RAD_TO_DEG(imu_data.gyroZ)  );
            putsUART1(buf);
            sprintf(buf,"temp = %0.3f C\n\r",imu_data.temp);
            putsUART1(buf);
            putsUART1("-------\n\r");
        }
    }
#endif
#ifdef DISP_FAST_SENSOR_DATA
    if (imu_i2c_status == I2C_STATUS_SUCCESFUL) {
        if(cnt % (FS/10) == 0) {
            sprintf(buf,"%5.2f, %5.2f, %5.2f,  %6.1f, %6.1f, %6.1f\n\r",\
                    imu_data.accX,imu_data.accY,imu_data.accZ,\
                    RAD_TO_DEG(imu_data.gyroX),RAD_TO_DEG(imu_data.gyroY),RAD_TO_DEG(imu_data.gyroZ));
            putsUART1(buf);
        }
        if (imu_data.overflow) RED_LED_ON();
        else RED_LED_OFF();
    } else {
        RED_LED_ON();
    }
#endif
#ifdef DISP_RAW_MEAN_DATA
    // Store mean datas for comparisons
    static UINT16 imu_mean_count = 0;
    static INT32 accXmean = 0,accYmean = 0,accZmean = 0;
    static INT32 gyroXmean = 0,gyroYmean = 0,gyroZmean = 0;
    if (read_raw_imu_data(&imu_raw_data) == I2C_STATUS_SUCCESFUL) {
        accXmean += ((INT32) imu_raw_data.accX);
        accYmean += ((INT32) imu_raw_data.accY);
        accZmean += ((INT32) imu_raw_data.accZ);
        gyroXmean += ((INT32) imu_raw_data.gyroX);
        gyroYmean += ((INT32) imu_raw_data.gyroY);
        gyroZmean += ((INT32) imu_raw_data.gyroZ);
        imu_mean_count++;
    }
    if (imu_mean_count == 256) {
        accXmean = accXmean/imu_mean_count; accYmean = accYmean/imu_mean_count;
        accZmean = accZmean/imu_mean_count; gyroXmean = gyroXmean/imu_mean_count;
        gyroYmean = gyroYmean/imu_mean_count; gyroZmean = gyroZmean/imu_mean_count;
        sprintf(buf,"aX = %d, aY = %d, aZ = %d\n\r",accXmean,accYmean,accZmean);
        putsUART1(buf);
        sprintf(buf,"gX = %d, gY = %d, gZ = %d\n\r",gyroXmean,gyroYmean,gyroZmean);
        putsUART1(buf);
        putsUART1("------------\n\r");
        accXmean = 0; accYmean = 0; accZmean = 0;
        gyroXmean = 0; gyroYmean = 0; gyroZmean = 0;
        imu_mean_count = 0;
    }
#endif
#ifdef POT_CONTROL
    potControl();
#endif


    if(cnt % (2*FS) == 0) {
        LED4_SWAP();
        //scanI2Cbus();
    }
    TS_PIN_LOW();
}





void scanI2Cbus(void) {
    UINT8 add, found=0;
    for (add = 0b0001000; add < 0b1111000; add++) {
        if (PokeAddress(add) == I2C_STATUS_SUCCESFUL) {
            found++;
            sprintf(buf,"Found I2C device at address 0x%x\n\r",add);
            putsUART1(buf);
        }
    }
    if (found == 0) {
        putsUART1("No devices found on the i2c line\n\r");
    } else {
        putsUART1("---\n\r");
    }
}

// Direct feedback control using a potentiometer
void potControl(void) {
//#define MODE_SPEED_FEEDBACK
#define MODE_DIRECT_VOLTAGE
    static UINT32 cnt = 0; // Incremented variables used for timing

    // Motor status variables
    static UINT8 motor1_status = MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED;
    static UINT8 motor_i2c_status = 1;

    // Check if button 1 is pressed, if enable/disable the motors
    static UINT8 but1_pressed = 0;
    static UINT32 prev_but1_cnt = 0;
    if (GET_BUT1() && !but1_pressed && (cnt-prev_but1_cnt > 5) ) {
        // Init/exit motor 1
        if (motor_i2c_status == I2C_STATUS_SUCCESFUL) {
            if( motor1_status != MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED) {
                set_calibration_status_unknown(NODE_ID);
                putsUART1("Motor 1 disabled\n\r");
            } else {
                calibrate_encoder_zero(NODE_ID);
                putsUART1("Motor 1 enabled\n\r");
            }
        } else {
            sprintf(buf,"Motor 1 with id %d not found on the bus\n\r",NODE_ID);
            putsUART1(buf);
        }


        // Debouncing and making sure the button only is pressed once
        // so holding it down doesn't call this function repeatedly
        prev_but1_cnt = cnt;
        but1_pressed = 1;
    } else if (!GET_BUT1() && but1_pressed && (cnt-prev_but1_cnt > 5)) {
        but1_pressed = 0;
        prev_but1_cnt = cnt;
    }

    // Read motor status
    LED5_ON();
    motor_i2c_status = get_status(NODE_ID,&motor1_status);
    LED5_OFF();

    // Send references to motors if they're enabled
    float send_value;
    if (motor_i2c_status == I2C_STATUS_SUCCESFUL && motor1_status != MOTOR_STATUS_ENCODER_CALIBRATION_NEEDED) {
        LED5_ON();
#if defined(MODE_SPEED_FEEDBACK)
        send_value = (((float)read_adc(A0))-512)*6.2831/512; // +- 360 deg/s
        motor_i2c_status = set_angular_velocity(NODE_ID,send_value);
#elif defined(MODE_DIRECT_VOLTAGE)
        send_value = (((float)read_adc(A0))-512)*24/512; // +- 24V
        motor_i2c_status = set_voltage(NODE_ID,send_value);
#else
        send_value = (((float)read_adc(A0))-512)*1.5708/512; // +- 90 degrees
        motor_i2c_status = set_angle(NODE_ID,send_value);
#endif
        LED5_OFF();
        RED_LED_ON();
    } else RED_LED_OFF();

    cnt++;
}