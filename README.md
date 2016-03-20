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

Some software examples are listed [here](./applications).

## Hardware Design
The hardware is divided into joints, links and end-effectors. 

The joints provide actuation and control electronics, allowing the user to control them in multiple ways. There are also be free-running joints, like pendulums, which allows for modeling and controlling complex systems. The joints designs are found [here](./joints).

Links are the connection elements between the joints and can be anything. In this application 20x20 mm aluminum profiles are used. [possible pics?].

End-effectors are devices meant to interact with the real world. They're attached to the end of the arm and can come in different shapes, like pens or claws. Some designs can be found [here](./end-effectors).

The manufacturing process used to fabricate the necessary parts are 3d-printing.

## The Simplified BSD License

Copyright (c) 2016, Viktor Kozma and David Broman

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

