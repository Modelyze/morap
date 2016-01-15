# MATLAB

Here's the MATLAB code used for different parts of the system

## Control Parameter Generation
To generate control parameters for a position controller run either `DCparamsPD.m` or `DCparamsPID.m` while inputting your motor data and inertia in the beginning of the file. If a speed controller is required run `DCparamsPI.m`. It will then output the struct that can be copy pasted into c-code.

The mupad scripts `poleplacement_PD.mn`, `poleplacement_PID.mn` and `poleplacement_PI_speed.mn` contains the mathematical derivation of these functions. Check [Viktors master thesis] to check the mathematical reasoning behind these derivations and implementations.

## Parameter Extraction
Run the application [data_logging](~/img) (fix link) and save the generated serial data. This data can then be translated into matlab readable data by running `read_log_file.m`. Then depending on the data the friction can be extracted by running `friction_extraction.m` and the inertia can be extracted by calling `inertia_extraction.m`.

An example parameter extraction is placed in `param_extraction.m` together with log files in order to visualize this workflow.

## Simulink scripts
The Simulink script `DCmotor.slx` can be used to test the position controllers (only really works for PD-controllers). Run `DCparamsPD.m` to generate controllers with different inputs and go for it.

`implementation.slx` tests code implementation of the controllers. This is mostly useful for controllers containing integrators as it contains an anti-windup system. 