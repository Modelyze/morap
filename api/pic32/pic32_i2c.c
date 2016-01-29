#ifndef PIC32_I2C_C
#define	PIC32_I2C_C


#define _SUPPRESS_PLIB_WARNING // For cleaness
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include <plib.h>
#include <xc.h>
#include "pic32_i2c.h"

UINT32 InitI2C(UINT32 system_clock, UINT32 i2c_clock)
{
    // Set clock and data (RG2, RG3) output high to remove reset errors:
    TRISGCLR = (1 << 2) | (1 << 3); // SET OUTPUT
    PORTGSET = (1 << 2) | (1 << 3); // SET HIGH
    PORTGCLR = (1 << 2) | (1 << 3); // SET LOW
    TRISGSET = (1 << 2) | (1 << 3); // SET BACK AS INPUT


    // Set the I2C baudrate
    //I2CConfigure(ACTIVE_I2C_BUS, I2C_EN);
    UINT32 actualClock = I2CSetFrequency(ACTIVE_I2C_BUS, system_clock, i2c_clock);
//    if ( abs(actualClock-i2c_clock) > i2c_clock/10 )
//    {
//        sprintf(buf,"Error: I2C1 clock frequency (%u) error exceeds 10%%.\n\r", (unsigned)actualClock);
//        putsUART1(buf);
//    }
    // Enable the I2C bus
    I2CEnable(ACTIVE_I2C_BUS, TRUE);

    return actualClock;
}

void ResetI2C() {
    I2CEnable(ACTIVE_I2C_BUS, FALSE); // Disable I2C module
    TRISGCLR = (1 << 2); // SET CLOCK AS OUTPUT
    int i,j;
    for (i = 0; i < 30; i++) {
        PORTGINV = (1 << 2); // Pulse clock
        for(j = 0; j < 100; j++); // small delay
    }
    PORTGCLR = (1 << 2); // Set low
    TRISGCLR = (1 << 3); // Set data as output
    PORTGSET = (1 << 3);
    PORTGCLR = (1 << 3); // line to clear eventual hold ups
    TRISGSET = (1 << 2) | (1 << 3); // Set back as inputs
    I2CEnable(ACTIVE_I2C_BUS, TRUE); // Reenable I2C module
}

BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(ACTIVE_I2C_BUS);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(ACTIVE_I2C_BUS) );

        if(I2CStart(ACTIVE_I2C_BUS) != I2C_SUCCESS)
        {
            //putsUART1("Error: Bus collision during transfer Start\n\r");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(ACTIVE_I2C_BUS);

    } while ( !(status & I2C_START) );

    return TRUE;
}


BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(ACTIVE_I2C_BUS));

    // Transmit the byte
    if(I2CSendByte(ACTIVE_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        return FALSE; // I2C Master Bus Collision
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(ACTIVE_I2C_BUS));

    return TRUE;
}

void StopTransfer( void )
{
    I2C_STATUS  status;

    // Send the Stop signal
    I2CStop(ACTIVE_I2C_BUS);

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(ACTIVE_I2C_BUS);

    } while ( !(status & I2C_STOP) );
}

/*
 * possible statuses
    0 - Success
    1 - NACK recieved during address transmission
    2 - NACK recieved during data transmission
    3 - Collision occured during start
    4 - Failed to transmit
    5 - I2C receiver overflow
 *
 */

UINT8 TransmitData(UINT8 address, UINT8* data, UINT8 datasize)
{
    UINT8 status = 0, idata;

    UINT8 SendAddress = address << 1 | 0; // Write
    // Start the transfer
    if( !StartTransfer(FALSE) )
    {
        // Error: Collision during start
        status = 3;
        return status;
    }
    // Transmit the address
    if (TransmitOneByte(SendAddress))
    {
        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(ACTIVE_I2C_BUS))
        {
            // Error: Sent address was not acknowledged
            status = 1;
            datasize = 0;
        }
    }
    else
    {
        // Error: Failed to transmit
        status = 4;
        datasize = 0;   // Don't bother
    }

    // Transmit the data
    for(idata = 0; idata < datasize; idata++){
        if (TransmitOneByte(data[idata]))
        {
            // Verify that the byte was acknowledged
            if(!I2CByteWasAcknowledged(ACTIVE_I2C_BUS))
            {
                // Error: Sent data byte was not acknowledged
                status = 2;
            }
        }
        else
        {
            // Error: Failed to transmit
            status = 4;
            datasize = 0;   // Don't bother
        }
    }

    // End the transfer
    StopTransfer();

    return status;
}

UINT8 ReadData(UINT8 address, UINT8* databuf, UINT8 datasize)
{
    UINT8 status = 0, idata;

    UINT8 SendAddress = address << 1 | 1;   // Read
    // Start the transfer
    if( !StartTransfer(FALSE) )
    {
        // Error: Collision during start
        status = 3;
        return status;
    }
    // Transmit the address
    if (TransmitOneByte(SendAddress))
    {
        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(ACTIVE_I2C_BUS))
        {
            // Error: Sent byte was not acknowledged
            status = 1;
            datasize = 0;
        }
    }
    else
    {
        // Error: Failed to transmit
        status = 4;
        datasize = 0;   // Don't bother
    }

    // READ DATA
    for(idata = 0; idata < datasize; idata++) {
        if(I2CReceiverEnable(ACTIVE_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
        {
            // Error: I2C Receive Overflow
            status = 5;
        }
        else
        {
            while(!I2CReceivedDataIsAvailable(ACTIVE_I2C_BUS));

            databuf[idata] = I2CGetByte(ACTIVE_I2C_BUS);

            if(idata < (datasize-1))
                I2CAcknowledgeByte(ACTIVE_I2C_BUS, TRUE);
            else
                I2CAcknowledgeByte(ACTIVE_I2C_BUS, FALSE);

            while (!I2CAcknowledgeHasCompleted(ACTIVE_I2C_BUS));
        }
    }

    // End the transfer
    StopTransfer();

    return status;
}

UINT8 PokeAddress(UINT8 address) {
    UINT8 status = 0;

    UINT8 SendAddress = address << 1 | 0; // Write
    // Start the transfer
    if( !StartTransfer(FALSE) )
    {
        // Error: Collision during start
        status = 3;
        return status;
    }
    // Transmit the address
    if (TransmitOneByte(SendAddress))
    {
        // Verify that the byte was acknowledged
        if(!I2CByteWasAcknowledged(ACTIVE_I2C_BUS))
        {
            // Error: Sent address was not acknowledged
            status = 1;
        }
    }
    else
    {
        // Error: Failed to transmit
        status = 4;
    }

    // End the transfer
    StopTransfer();

    return status;
}


#endif