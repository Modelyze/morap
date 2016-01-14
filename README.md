# MORAP: a Modular Robotic Arm Platform

### Contributors
* Viktor Kozma, KTH Royal Institute of Technology, vkozma@kth.se
* David Broman, KTH Royal Institute of Technology, dbro@kth.se

## Introduction
The MORAP system is designed to be a modular robotic arm for research purposes. It's modular in a way that modules can be designed and added or interchanged to achieve new configurations.

It is designed together with a Modelyze modeling library that allows building and simulating arm configurations, giving the user the ability to design and evaluate designs in software.

## Overall Design
The system is designed as a distributed system where each component is communicating on a standardized central data bus. This means that it's trivial to add other modules or sensors to the system.

The system has one master which controls all of the individual sub-systems and handles communication with the user. This master controller can be anything, a microcontroller, Arduino, Raspberry pi etc. The user programs this controller to fulfill specific tasks relevant to their intended application.

## Software Design
For actuation each joint have built in control electronics and tuning algorithms, providing an intuitive and easy way to control the system. The communication protocol is listed in [here](./api/) and an API implementation on a PIC32 microcontroller is listed in section [here](./api/pic32/).

Some software examples are listed in [link to applications].

## Hardware Design
The hardware is divided into joints, links and end-effectors. 

The joints provide actuation and control electronics, allowing the user to control them in multiple ways. There are also be free-running joints, like pendulums, which allows for modeling and controlling complex systems. The joints designs are found here [link to joints folder].

Links are the connection elements between the joints and can be anything. In this application 20x20 mm aluminum profiles are used. [possible pics?].

End-effectors are devices meant to interact with the real world. They're attached to the end of the arm and can come in different shapes, like pens or claws. Some designs can be found in [link to end-effector folder].
