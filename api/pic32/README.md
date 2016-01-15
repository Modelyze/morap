# API for a PIC32 Microcontroller
This is an api with an implementation of the motors on a pic32 microcontroller.

This api contains two parts, I2C drivers for the PIC32 and drivers for using this library to communicate with the motors. It also contains drivers for individual end-effectors.

## How to Use
Include ```modular_arms.h```  into your project:
```c
#include "modular_arms.h"
```
Initiate the arm by calling:
```c
init_arm(GetPeripheralClock());
```
Where ```GetPeripheralClock()``` is a macro defining the peripheral clock speed in Hz.

When the arm is initiated you can start issuing commands to it.

Before the arm can start being controlled the encoders of each individual motors needs to be calibrated so they know what physical direction corresponds to what angle. This is done by (for each motor on the arm) calling:
```c
calibrate_encoder_zero(node_id);
```
Where ```node_id``` is a variable corresponding to the id-number of the motor you want to calibrate. Each individual motor has it's unique id ranging from 0 to 7. When calling this the node assumes that you have physically oriented the motors to point in the direction you want to define as having an angle of 0 radians.

When all the motors are calibrated they can start being used. There are several ways to control these motors, the most useful being the position control. To position a motor at a specific angle you can call:
```c
set_angle(node_id,target_angle);
```
Where ```target_angle``` is the angle radians at which the motor positions itself.

There's also a built in speed feedback controller which can be called by:
```c
set_angular_velocity(node_id,target_angular_speed);
```
Where ```target_angular_speed``` is the angular speed in radians/second which the motor will rotate at.

You can read variables and status messages from the node in a similar way. For example to get the current angle of the motor you can call:
```c
float current_angle;
get_angle(node_id,&current_angle);
```
This allows you to monitor the current motor position, useful when using the speed input to avoid going out of bounds.

When done controlling the arm the motors can be disabled by calling:
```c
disable_motor(node_id);
```
This turns them of and allows you to rotate the motor freely. 

To re-enable the motors just run a ```set_angle``` or ```set_angular_velocity``` command and it will enable the motors and fulfill that command.

If you don't want the motor to rotate at all when disabled a brake can be activated by calling:
```c
enable_brake(node_id);
```
This shorts the leads of the motors, providing high resistance against turning. This will automatically be disabled when the motors are re-enabled, forcing you to reactivate it the next time they're disabled.

Alternatively the motors can be turned of by setting the encoder status of the motors to unknown by calling:
```c
set_calibration_status_unknown(node_id);
```
This requires you to recalibrate the encoders when activating it again later.


## Tuning the control algorithm
To improve the performance of the feedback controller it sometimes has to be tuned for its intended applications. As the default programmed controller on each of the nodes are optimized around a low inertia they may not operate to their full capacity when they drive a high inertia. So for applications where a motor is supposed to drive a high inertia (for example the first joint of an arm) the controller can be tuned by calling:
```c
tune_control_params(node_id,TUNING_PD_POSITION_CONTROLLER,TUNING_MEDIUM_INERTIA,TUNING_POSITION_POLE_MEDIUM);
delay(50); // small delay so the controller has time to calculate the new variables
unsigned char prog_status;
get_control_prog_status(node_id, &prog_status);
if (prog_status == MOTOR_CONTROL_PROGRAMMING_STATUS_SUCCESS) {
  //SUCCESS
}
```
Some common inertias are predefined and should serve well.

Two different position controllers are available for tuning, these are:
```
TUNING_PD_POSITION_CONTROLLER
TUNING_PID_POSITION_CONTROLLER
```
For most positioning tasks a PD controller is sufficient and more robust than a PID-controller. However certain situations, like vertically mounted arms, requires a PID-controller to get rid of steady state errors. 

To tune the speed controller you can similarily call:
```c
tune_control_params(node_id,TUNING_PI_SPEED_CONTROLLER,TUNING_MEDIUM_INERTIA,TUNING_SPEED_POLE_MEDIUM);
delay(50); // small delay so the controller has time to calculate the new variables
unsigned char prog_status;
get_control_prog_status(node_id, &prog_status);
if (prog_status == MOTOR_CONTROL_PROGRAMMING_STATUS_SUCCESS) {
  //SUCCESS
}
```
This will tune the speed controller that is called when using the command ```set_angular_velocity()```, allowing you to fine tune responses.

The default controller parameters can be recalled using the command:
```c
set_default_control_params(node_id);
```


## Compatability
This api has been developed for a the PIC32MX320F128 microcontroller equipped on a uno32, an arduino compatible development board. It should work on similar microcontrollers but it hasn't been tested.