# Power Shield for UNO32
A power shield designed to fit on an UNO32 designed to take input from two 4 mm lab cables, convert it to 5V and 3.3V and outputs it to two 6-pin connectors compatible with the standard connectors on the motor control chips.

Contains both Gerber and eagle source files

## Functions
* Inputs selectable voltage Vcc (>6V) through two 4 mm lab cables (Vcc and GND)
* Converts Vcc to 5V and 3.3V using two switching regulators
* Outputs Vcc, 5V, 3.3V, GND, SCL, SDA through a system standard 6 pin connector
* Contains pullup resistors for SCL and SDA line
* Contains two buttons connected to UNO32 pins 2 & 7 (RD8 & RD9)
* Contains three leds on UNO32 pins 3, 5 & 6 (RD0, RD1 & RD2) and one power LED.

## Schematic
![schematic](./power_shield_schematic.png)

## Bill of Material
n | value | device | package
---: | :--- | :--- | :---
1x | LM2576-5.0 | Switch Regulator 5V | TO-220-5
1x | LM2576-3.3 | Switch Regulator 3.3V | TO-220-5
2x | 1000&mu;F | Electrolytic Capacitor | 10mm Electrolytic Capacitors SMD
2x | 100&mu;F | Electrolytic Capacitor | 10mm Electrolytic Capacitors SMD
2x |  | Diode | DO-214AA=SMB
2x | 0.1mH (+1.5A) | Inductor | Radial 5mm pitch
2x | RED | LED | CHIPLED_1206
1x | GREEN | LED | CHIPLED_1206
1x | YELLOW | LED | CHIPLED_1206
1x | 1k&Omega; | Resistor | R0603
3x | 470&Omega; | Resistor | R0603
2x | 3.3k&Omega; | Resistor | R0603
2x | 10k&Omega; | Resistor | R0603
2x | | Pushbutton | Through-Hole PCB-switch
2x | 2x8 pin | Socket Header | 2.54mm pitch, long legged socket header
1x | 2x6 pin | Socket Header | 2.54mm pitch, long legged socket header
1x | 1x6 pin | Socket Header | 2.54mm pitch, long legged socket header
2x | MOLEX 3.96mm 6p | male connector | MOLEX_3.96MM_6P
1x | RED | Laboratory Socket | 15.3mm pitch PCB
1x | BLACK | Laboratory Socket | 15.3mm pitch PCB