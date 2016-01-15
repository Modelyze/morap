% PD-position control parameters
clear all; close all; clc

% Different motors

% % DCX19S
% % Electrical and mechanical
% R = 5.73 + 0.33; L = 0.329e-3; K = 0.0178; m_motor = 0.122;
% % Gearbox parameters
% n = 326700/1900; eta = 0.74; c = 0.1;
% % Limitations and poles
% imax = 1.0; umax = 24; w = -6;

% DCX22L
% Electrical and mechanical
R = 1.83 + 0.33; L = 0.192e-3; K = 0.0229; m_motor = 0.165;
% Gearbox parameters
n = 300564.0/1300.0; eta = 0.74; c = 0.1;
% Limitations and poles
imax = 2.5; umax = 24; w = -12;

% % DCX26L
% % Electrical and mechanical
% R = 0.74 + 0.33; L = 0.129e-3; K = 0.0214; m_motor = 0.412;
% % Gearbox parameters
% n = 328509/2197; eta = 0.75; c = 0.05;
% % Limitations and poles
% imax = 1.5; umax = 24; w = -9; % w = -12, -9

% Friction modeling
dv = 1e-5; % deadband
k_f = [0.072091, 0.040963, -0.003493]; % Nonlinear friction [static, linear, quadratic]
%k_f = [0 c 0]; % Linear friction

% Inertias
J = 0.05;
%J = 0.2;

%%%%%%%%%%%%%%%%%%%%% MATH BELOW %%%%%%%%%%%%%%%%%%%%%%%%

disp(['Max speed: ',num2str(umax/(K*n) * 30/pi,'%0.1f'),' rpm']);

% transfer function stuff:
s = tf('s');
Gs = minreal(eta*n*K/(R*J*s^2 + (eta*n*n*K*K+R*c)*s));
disp(['Pole(s) for the motor in open loop: ',num2str(nonzeros(pole(Gs)))])
[NUM,DEN] = tfdata(Gs);
g0 = NUM{1}(3); g1 = DEN{1}(2); clear NUM DEN
J_design = J;   % The inertia the controller was designed around


% Pole placement for output feedback
w1 = w + 1i; w2 = w - 1i; 
w3 = min(nonzeros(pole(Gs)));   % Poles, w1 + w2 will decide dynamics

r0 = -g1 - w1 - w2 - w3;
s0 = -(w1*w2*w3)/g0;
s1 = (g1*w1 + g1*w2 + g1*w3 + w1*w2 + w1*w3 + w2*w3 + g1^2)/g0;

Sc = s1*s + s0;
Rc = r0 + s;
t0 = w1*w2/g0; Tc = t0*(s - w3);
Ac = s^2 + g1*s;
Bc = g0 + 0*s;

Gc = minreal(Bc*Tc/(Ac*Rc + Bc*Sc));

if (any(zero(Rc) > 0))
    disp('WARNING, Rc contributes with positive poles!\n');
end
fprintf('Recommended Fs: %0.1f - %0.1f Hz\n',...
    (10*max(abs([w1,w2,w3])))/2/pi,(30*max(abs([w1,w2,w3])))/2/pi );


% Discrete controller
enc_res = 2*pi/(128*n);  % Encoder resolution
volt_res = 48/(128*4); % Voltage resolution from PWM
Ts = 1 / 320;
z = tf('z',Ts);

% Tustin:
Gdff = minreal(((2*t0 - Ts*t0*w3)*z - 2*t0 - Ts*t0*w3)/((Ts*r0 + 2)*z + Ts*r0 - 2))
%c2d(Tc/Rc,Ts,'zoh')
Gdfb = minreal(((2*s1 + Ts*s0)*z + Ts*s0 - 2*s1)/((Ts*r0 + 2)*z + Ts*r0 - 2))
%c2d(Sc/Rc,Ts,'zoh')


% Low pass filter on current limiter:
tau_cllp = 4*Ts;
a_cllp = Ts/(tau_cllp+Ts);
Gdcllp = (z*a_cllp)/(z - (1-a_cllp));


% Prints continous time parameters
fprintf('s1 = %0.3f, s0 = %0.3f, r0 = %0.3f\n',s1,s0,r0);

% Clears up the workspace
clear r0 s0 s1 w1 w2 w3 t0 g0 g1

Gff = Tc/Rc
Gfb = Sc/Rc
clear Gff Gfb
% Prints out control struct for software implementation
%print_control(0,umax,imax,K,n,R,Gdff,Gdfb);
fprintf('\nControl parameter struct:\n');
print_control_struct( Gdff, Gdfb )


