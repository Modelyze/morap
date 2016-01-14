/* 
 * File:   pic32_i2c.h
 * Author: Viktor Kozma
 *
 * defines the i2c functions for uno32 as a master
 *
 *
 * Created on May 7, 2015
 */

#ifndef PIC32_I2C_H
#define	PIC32_I2C_H

#define ACTIVE_I2C_BUS              I2C1

/*******************************************************************************
  Function:
    UINT32 InitI2C(UINT32 system_clock, UINT32 i2c_clock);

  Summary:
     Initiates the i2c bus as a master

  Description:
     This routine initializes the i2c bus with the chosen i2c_clock baud rate.
     It returns the actual clock the system works at that can be compared to
     the desired one to check for errors.

  Precondition:
     None.

  Parameters:
     system_clock  - the current system peripheral clock frequency
     i2c_clock     - the desired i2c baud rate

  Returns:
     The actual clock resulted from the i2c bus

  Example:
    <code>
    UINT32 actual_clock = InitI2C(get_peripheral_clock(), I2C_CLOCK_FREQ);
    </code>

  Remarks:
     This only needs to be called once
  *****************************************************************************/
UINT32 InitI2C(UINT32 system_clock, UINT32 i2c_clock);

/*******************************************************************************
  Function:
    BOOL StartTransfer( BOOL restart )

  Summary:
    Starts (or restarts) a transfer to/from the active bus.

  Description:
    This routine starts (or restarts) a transfer on the active bus, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition

  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling

  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/
BOOL StartTransfer( BOOL restart );

/*******************************************************************************
  Function:
    BOOL TransmitOneByte( UINT8 data )

  Summary:
    This transmits one byte to the EEPROM.

  Description:
    This transmits one byte to the EEPROM, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
  *****************************************************************************/
BOOL TransmitOneByte( UINT8 data );

/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the active bus.

  Description:
    This routine Stops a transfer to/from the active bus, waiting (in a
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
  *****************************************************************************/
void StopTransfer( void );

/*******************************************************************************
Function:
    UINT8 TransmitData(UINT8 address, char* data, UINT8 datasize)
    Summary:
    Transfers the given data to a given address

Description
    This function sends the given amount of data to the address.
    Returns a status regarding the result.

Precondition:
    The I2C module must have been initialized

Parameters:
    address: The 7-bit address to send to
    data: A pointer to an array containing the data
    datasize: The length of the data array (not including the address)

Returns:
    0 - Success
    1 - NACK recieved during address transmission
    2 - NACK recieved during data transmission
    3 - Collision occured during start
    4 - Failed to transmit


Example:
    <code>
        UINT8* sendData[3];
        sendData[0] = 0x05;
        sendData[1] = 0xA3;
        sendData[2] = 0xAA;
        UINT8 result = TransmitData(0x50,sendData,3);
    </code>

  Remarks:
    This is a blocking routine that polls during its execution
 ******************************************************************************/
UINT8 TransmitData(UINT8 address, UINT8* data, UINT8 datasize);

/*******************************************************************************
Function:
    UINT8 ReadData(UINT8 address, char* data, UINT8 datasize)
Summary:
    Reads the I2C data from a given address

Description
    This function requests data from a given address and and places it in the
    provided buffer.

Precondition:
    The I2C module must have been initialized

Parameters:
    address: The 7-bit address to send to
    data: A pointer to an array in which to store the recieved data
    datasize: The amount of bytes you want to recieve

Returns:
    0 - Success
    1 - NACK recieved during address transmission
    2 - NACK recieved during data transmission
    3 - Collision occured during start
    4 - Failed to transmit
    5 - I2C receiver overflow


Example:
    <code>
        UINT8* recData;
        UINT8 result = TransmitData(0x50,recData,1);
    </code>

  Remarks:
    This is a blocking routine that polls during its execution
 ******************************************************************************/
UINT8 ReadData(UINT8 address, UINT8* databuf, UINT8 datasize);



/*******************************************************************************
Function:
    UINT8 PokeAddress(UINT8 address)
    Summary:
    Pokes an I2C address to see if it responds

Description
    Pokes the provided I2C address to see if the device responds. Returns
    success or failures

Precondition:
    The I2C module must have been initialized

Parameters:
    address: The 7-bit address to send to

Returns:
    0 - Success
    1 - NACK recieved during address transmission
    3 - Collision occured during start
    4 - Failed to transmit


Example:
    <code>
        UINT8 result = PokeAddress(0b1101001);
    </code>

  Remarks:
    This is a blocking routine that polls during its execution
 ******************************************************************************/
UINT8 PokeAddress(UINT8 address);


#endif	/* PIC32_I2C_H */

