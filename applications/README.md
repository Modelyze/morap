# Applications using a PIC32 microcontroller

Some example applications using the PIC32 API. These applications are
implemented and tested on the ChipKIT UNO32 development board.

## Available applications
The following applications can be opened as MPLAB projects:

[Potentiometer control](./potentiometer_control) is a simple
application where a button can be used to turns on/off one or two
motors. Each motor can be controlled by a separate potentiometer.

[Draw test](./draw_test) performs a simple control sequence in which
it positions an optional pen end-effector at three different
angles. This allows the user to evaluate the repeatability of the
system and to identify potential drifts in the encoders. If two motors are
found on the system, it will use both for the control sequence.

[Data logger](./data_logger) generates log data that can be used for
parameter extraction.  When using the application, save the log data
data and run the appropriate [matlab
scripts](../joints/dc-motor-joint/software/matlab).

[Inverse kinematics](./inverse_kinematics) is an application for
controlling a 2D robotic arm with two joints. The user supplies an x-y
coordinate and it calculates the required angles to reach that
position. It also comes with an preliminary implementation of a path
following algorithm in which the user can define a path that the arm
should follow. This application is still work in progress.

[Inverted pendulum](./pendulum) is a work-in-progress application that tries to
balance an inverted pendulum. It utilizes a
MPU9150 IMU for measuring the angle of the pendulum and is
controlled by a state-feedback controller.

In most of the applications you can change some definitions to change the
identifiers of the target motors or the arm lengths. More information is available at the beginning of each main source file.

## Installation

To run the applications above, you need to download and install the following development environment and compiler:

* [MPLAB IDE](http://www.microchip.com/mplab/mplab-x-ide). This is the main development environment for the PIC32 microprocessors from Microchip.
* [XC32 compiler](http://www.microchip.com/mplab/compilers). The main cross compiler. There is a free compiler version available.
* [PIC32 Peripheral Library](http://www.microchip.com/SWLibraryWeb/product.aspx?product=PIC32%20Peripheral%20Library). This library is necessary if you use XC32 compiler 1.4 or later.
