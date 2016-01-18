# APIs for Controlling the Robotic Arm

This section contains the API for controlling the robotic arm as well as information on how it works so it can be ported to other platforms.

## Available Applications
An API has been developed for a PIC32 processor, or more specifically the PIC32MX320F128 equipped on a uno32, an arduino compatible development board. This API can be found [here](./pic32).

## How it Works
The communication protocol works on an I2C bus [link to wiki?], a serial communication protocol, running at 100 kHz. The I2C protocol is a single master protocol in which a master can initiate communication and write or request data from an addressable slave. The system can address up to 128 different units (but in practice some of these are reserved resulting in 112 practical addresses), allowing for many different subsystems to be included on the bus. 

Each joint and end-effector has an unique address allowing the master to address individual modules. If the addressed slave exists and acknowledges its existence on the bus the master starts to write data to the slave one byte at a time. The master can also request data from a slave. However to define what data it wants to read the master usually has to write that info to the slave before requesting data from it.

As this is a standardized protocol peripheral hardware, for example sensors, can be easily included in the system.

## Communication Protocol
In order for the slave to know what type of data the master writes the first byte sent after the address is an identifier. For some commands, like disabling the motors or calibrating the encoders, only requires this identifier to take effect. If additional data is needed, for example setting reference points for the controller, it is provided after the identifier with data-types larger than one byte sent with LSB first [check].

A table containing the identifiers, their purpose and format (purpose(variable type)(nr of bytes)) (the identifier has the variable type (uint8)) are located below:

id | purpose | format 
---------: | :---------- | :----------- 
0   | disable motors | id(1) 
1   | set position reference point | id(1)-pos_ref(float)(4) 
2   | set speed reference point | id(1)-speed_ref(float)(4) 
3   | set drive voltage | id(1)-voltage(float)(4) 
14  | disable brake | id(1) 
15  | enable brake | id(1) 
16  | calibrate encoder to zero | id(1) 
17  | calibrate encoder to supplied angle | id(1)-angle(float)(4) 
31  | set encoder calibration status to unknown | id(1) 
32  | program new control parameters | see next section 
33  | set control parameters to default values | id(1) 
34  | tune control parameters | see next section 
128 | next read operation will return the angle (float(4)) | id(1) 
129 | next read operation will return the position reference (float(4)) | id(1) 
130 | next read operation will return the speed reference (float(4)) | id(1) 
131 | next read operation will return the motor status (byte(1)) | id(1) 
132 | next read operation will return the controller programming status (byte(1)) | id(1) 
133 | next read operation will return the applied voltage (float(4)) | id(1) 
134 | next read operation will return the current (float(4)) | id(1) 
136 | next read operation will return the address of the node (byte(1)) | id(1) 

Potential returns for the motor status are as follows:

return value | meaning
---------: | :---------
1 | enabled
2 | disabled
3 | needs encoder calibration

Potential returns for the controller programming status are:

return value | meaning
----------: | :------------
1 | nothing has been programmed
2 | programming successful
3 | programming failed

## Programming the Control Variables
The controller is described in the [matlab](../joints/dc-motor-joint/software/matlab) section. There are three different ways to program the controller with new control parameter.

Send the control parameters directly (id = 32). This is done by sending all the parameters containted in the discrete controller directly. The format looks like this: 
```
id(1) - cMode(uint8)(1) - Fs(uint16)(2) - nd(uint8)(1) - d(float[nd])(nd*4) - 
  nc(uint8)(1) - c(float[nc])(nc*4) - nf(uint8)(1) - f(float[nf])(nf*4) - 
  I(float)(4) - checksum(uint8)(1)
```

The checksum is calculated by the following psuedocode:
```c
unsigned int sum = sum(control_params); // the sum of all bytes sent (excluding the id)
while (sum >> 8)
  sum = (sum >> 8) + (sum & 0xFF);
unsigned char checksum = ~((unsigned char) sum);
```

This approach is useful when you want to really know what each joint are doing, for example when performing simulations that should be compared to reality.

The motor node can also tune themselves when provided with tuning variables (id = 34). The formatting of this message looks like:
```
id(1) - controller_type(uint8)(1) - inertia(float)(4) - closed_loop_pole(float)(4)
```

The controller type can be be one of the following values:

| value | controller |
| ---: | :--- |
| 0 | PD position controller |
| 1 | PID position controller |
| 2 | PI speed controller |

The inertia is the inertia that the controller designs itself around.

The closed loop pole is the pole of the controller which determines how fast the controller is. It should be < 0 and the smaller it is the faster the controller becomes.

After performing any of these commands read the programming status after to ensure that they where programmed successfully.