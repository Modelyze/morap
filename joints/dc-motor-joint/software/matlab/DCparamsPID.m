% PID-position control
clear all; close all; clc

% % DCX22L
% % Electrical and mechanical
% R = 1.83 + 0.33; L = 0.192e-3; K = 0.0229; m_motor = 0.165;
% % Gearbox parameters
% n = 300564.0/1300.0; eta = 0.74; c = 0.1;
% % Limitations and poles
% imax = 1.5; umax = 24; w = -6;

% DCX26L
% Electrical and mechanical
R = 0.74 + 0.33; L = 0.129e-3; K = 0.0214; m_motor = 0.412;
% Gearbox parameters
n = 328509/2197; eta = 0.75; c = 0.05;
% Limitations and poles
imax = 2; umax = 24; w = -6; % w = -12, -9

% Friction modeling
dv = 1e-5; % deadband
k_f = [0.072091, 0.040963, -0.003493]; % Nonlinear friction [static, linear, quadratic]
%k_f = [0 c 0]; % Linear friction

% Load inertia
J = 0.05;
disp(['Max speed: ',num2str(umax/(K*n) * 30/pi,'%0.1f'),' rpm']);

% transfer function of dc-motor:
s = tf('s');
Gs = minreal(eta*n*K/(R*J*s^2 + (eta*n*n*K*K+R*c)*s));
disp(['Pole(s) for the motor in open loop: ',num2str(nonzeros(pole(Gs)))])
[NUM,DEN] = tfdata(Gs);
g0 = NUM{1}(3); g1 = DEN{1}(2); clear NUM DEN
J_design = J;   % The inertia the controller was designed around

% Pole placement for PID-controller and output feedback
w1 = w + 1i; w2 = w - 1i; 
w3 = min(nonzeros(pole(Gs)));   % Poles, w1 + w2 will decide dynamics
w4 = w3; % Double pole

r0 = -g1-w1-w2-w3-w4;
s0 = (w1*w2*w3*w4)/g0;
s1 = -(w1*w2*w3 + w1*w2*w4 + w1*w3*w4 + w2*w3*w4)/g0;
s2 = (g1*w1 + g1*w2 + g1*w3 + g1*w4 + w1*w2 + w1*w3 + w1*w4 + w2*w3 + w2*w4 + w3*w4 + g1*g1)/g0;

Sc = s2*s^2 + s1*s + s0;
Rc = s*(r0 + s);
t0 = w1*w2/g0; Tc = t0*((s-w3)*(s-w4));
Ac = s^2 + g1*s;
Bc = g0 + 0*s;

if (any(zero(Rc) > 0))
    disp('WARNING, Rc contributes with positive poles!');
end
fprintf('Recommended Fs: %0.1f - %0.1f Hz\n',...
    (10*max(abs([w1,w2,w3,w4])))/2/pi,(30*max(abs([w1,w2,w3,w4])))/2/pi );


% Discrete controller
enc_res = 2*pi/(128*n);  % Encoder resolution
volt_res = 48/(128*4); % Voltage resolution from PWM
Ts = 1 / 625;
z = tf('z',Ts);


% c2d approach
%Gdff = c2d(Tc/Rc,Ts,'zoh')
%Gdfb = c2d(Sc/Rc,Ts,'zoh')

% Tustin approach
o1 = Ts*Ts*w1*w2*w3*w4;
o2 = 2*Ts*w1*w2*w4;
o3 = 2*Ts*w1*w2*w3;
Gdfb = minreal(((s0*Ts*Ts + 2*s1*Ts + 4*s2)*z^2 + (2*Ts*Ts*s0 - 8*s2)*z + (s0*Ts*Ts - 2*s1*Ts + 4*s2))/((2*Ts*r0 + 4)*z^2 + (-8)*z + (4 - 2*Ts*r0)));
%c2d(Sc/Rc,Ts,'zoh')
Gdff = minreal(((4*w1*w2-o3-o2+o1)*z^2 + (2*Ts*Ts*w1*w2*w3*w4 - 8*w1*w2)*z + (4*w1*w2+o3+o2+o1))/((4*g0 + 2*Ts*g0*r0)*z^2 + (-8*g0)*z + (4*g0-2*Ts*g0*r0)));
%c2d(Tc/Rc,Ts,'zoh')
clear o1 o2 o3

% Seperate integrator version
NUM = tfdata(Tc); t2 = NUM{1}(1); t1 = NUM{1}(2); t0 = NUM{1}(3);
if abs(t0 - s0) > 1e-6, fprintf('----- wonky shit'); end;
clear NUM
sf1 = s2; sf0 = s1 - s0/r0;
tf1 = t2; tf0 = t1 - t0/r0;
If = s0/r0;
Sfc = sf1*s + sf0; Tfc = tf1*s + tf0; Rfc = s + r0;
Gfdfb = minreal(((2*sf1 + Ts*sf0)*z + Ts*sf0 - 2*sf1)/((Ts*r0 + 2)*z + Ts*r0 - 2))
Gfdff = minreal(((2*tf1 + Ts*tf0)*z + Ts*tf0 - 2*tf1)/((Ts*r0 + 2)*z + Ts*r0 - 2))
%Ifd = minreal(((If*Ts)*z + If*Ts)/(2*z - 2))
Ifd = minreal((If*Ts*z)/(z-1))
clear sf1 sf0 tf1 tf0

% Low pass filter on current limiter:
tau_cllp = 4*Ts;
a_cllp = Ts/(tau_cllp+Ts);
Gdcllp = (z*a_cllp)/(z - (1-a_cllp));

% Prints continous time parameters
%fprintf('s1 = %0.3f, s0 = %0.3f, r0 = %0.3f\n',s1,s0,r0);

% Clears up the workspace
clear r0 s0 s1 s2 w1 w2 w3 w4 t2 t1 t0 g0 g1

% Gff = Tc/Rc
% Gfb = Sc/Rc
% clear Gff Gfb
% Prints out control struct for software implementation
print_control_struct( Gfdff, Gfdfb, Ifd, 0 )
print_control(0,umax,imax,K,n,R,Gfdff,Gfdfb,Ifd);