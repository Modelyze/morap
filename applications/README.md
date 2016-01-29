# Applications of the Modular Arm on an PIC32
Some example applications of the modular arm api on a PIC32. Especially the one equipped on an uno32 development board from chipkit.

## Available applications
Some applications are provided here that can be opened as an MPLAB project.

[Potentiometer control](./potentiometer_control) is a simple application in which pressing a button turns on/off up to two motors. When they're on each motor can be controlled by a separate potentiometer.

[Draw test](./draw_test) performs a simple control sequence in which it positions an optional pen end-effector at three different angles and paints one dot. This allows the user to evaluate the repeatability of the system and identify eventual drift in the encoders. If two motors are found on the system it will use both for the control sequence.

[Data logger](./data_logger) is used to generate logging data used for parameter extraction. Log the data arriving from the terminal and run the appropriate [matlab scripts](../joints/dc-motor-joint/software/matlab).

[Inverse kinematics](./inverse_kinematics) is an initial application for a 2d robotic arm with two joints in which the user supplies an x-y coordinate and it calculates the required angles to reach that position. It also comes with an preliminary implementation of a path following algorithm in which the user can define an path which the arm attempts to follow. It still needs work to function properly but can be used to draw simple shapes.

[Pendulum](./pendulum) is an in progress application which attempts to balance an inverted pendulum at the end of one motor. It utilizes a mpu9150 imu for measuring the angle of said pendulum and can then be controlled by an adequate control algorithm.

In most application you can change some definitions to change the identifiers of the target motors or the arm lengths etc. Refer to the top their respective main files for information of stuff you can change.