# PIC24 Motor Node Source Code

Here the source code for the motor node is located.

## EEPROM considerations
As all nodes run on the same source code their individual parameters needs to be programmed into their respective EEPROM memories. These parameters includes:

* Motor id, used for determining their I2C-address
* Motor data parameters, used for current limiting and control tuning
* Default position control parameters
* Default speed control parameters

So before programming it with the [node](./pic24-node) you first need to run the [eeprom programming program](pic24-eeprom-programming) with your default parameters defined.

This means that you might have to set the programmer to preserve the internal eeprom of the target device when programming it. In MPLAB with a PICKIT3 this is done by: Run -> Set Project Configuration -> Customize -> PICKIT3 -> Conserve EEPROM Memory
