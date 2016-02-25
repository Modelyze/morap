% PI-speed control parameters
clear all; close all; clc

% % DCX22L
% % Electrical and mechanical
% R = 1.83 + 0.33; L = 0.192e-3; K = 0.0229; m_motor = 0.165;
% % Gearbox parameters
% n = 300564.0/1300.0; eta = 0.74; c = 0.1;
% % Limitations and poles
% imax = 1.5; umax = 24; w = -60;

% DCX26L
% Electrical and mechanical
R = 0.74 + 0.33; L = 0.129e-3; K = 0.0214; m_motor = 0.412;
% Gearbox parameters
n = 328509/2197; eta = 0.75; c = 0.05;
% Limitations and poles
imax = 2; umax = 24; w = -50; % w = -12, -9

% Friction modeling
dv = 1e-5; % deadband
k_f = [0.072091, 0.040963, -0.003493]; % Nonlinear friction [static, linear, quadratic]
%k_f = [0 c 0]; % Linear friction

% Load inertia
J = 0.1;
disp(['Max speed: ',num2str(umax/(K*n) * 30/pi,'%0.1f'),' rpm']);

% transfer function of dc-motor:
s = tf('s');
Gs = minreal(eta*n*K/(R*J*s + (eta*n*n*K*K+R*c)));
disp(['Pole(s) for the motor in open loop: ',num2str(nonzeros(pole(Gs)))])
[NUM,DEN] = tfdata(Gs);
g0 = NUM{1}(2); g1 = DEN{1}(2); clear NUM DEN
J_design = J;   % The inertia the controller was designed around

% Pole placement for output feedback
w1 = w; 
w2 = min(nonzeros(pole(Gs)));
w3 = min(nonzeros(pole(Gs)));   % Poles, w1 will decide dynamics

r0 = -g1 - w1 - w2 - w3;
s0 = -(w1*w2*w3)/g0;
s1 = (g1*w1 + g1*w2 + g1*w3 + w1*w2 + w1*w3 + w2*w3 + g1^2)/g0;

Sc = s1*s + s0;
Rc = s*(r0 + s);
t0 = -w1/g0; Tc = t0*(s - w3)*(s - w2);
Ac = s + g1;
Bc = g0 + 0*s;

if (any(zero(Rc) > 0))
    disp('WARNING, Rc contributes with positive poles!');
end
fprintf('Recommended Fs: %0.1f - %0.1f Hz\n',...
    (10*max(abs([w1,w2,w3])))/2/pi,(30*max(abs([w1,w2,w3])))/2/pi );

Gc = minreal(Bc*Tc/(Ac*Rc + Bc*Sc)); % closed loop

% PID/PD-parameters for cascaded position control
k = abs(w);
%%% PD
% wcp1 = -6;
% cpP = -wcp1; cpI = 0; cpD = -wcp1/k;
%%% PID
wcp1 = -6+1i; wcp2 = -6-1i; 
cpP = -(k*wcp1 + k*wcp2 -wcp1*wcp2)/k;
cpI = wcp1*wcp2;
cpD = -(wcp1 + wcp2)/k;
clear wcp1 wcp2 k
% Traj planner time to w_max function
w_traj = @(th,t,a) [t*a/2+sqrt((t*a/2)^2 - th*a), t*a/2-sqrt((t*a/2)^2 - th*a)];

% Discrete controller
enc_res = 2*pi/(128*n);  % Encoder resolution
volt_res = 48/(128*4); % Voltage resolution from PWM
Ts = 1/250; %1 / 625;
z = tf('z',Ts);


% c2d approach
Gdff = c2d(Tc/Rc,Ts,'zoh');
Gdfb = c2d(Sc/Rc,Ts,'zoh');


% Seperate integrator version
NUM = tfdata(Tc); t2 = NUM{1}(1); t1 = NUM{1}(2); t0 = NUM{1}(3);
sf0 = s1 - s0/r0;
tf1 = t2; tf0 = t1 - t0/r0;
If = s0/r0;
Sfc = sf0 + 0*s;
Tfc = tf1*s + tf0;
if abs(t0 - s0) > 1e-6, fprintf('----- wonky shit'); end;
clear NUM
Rfc = (s + r0);
% % c2d approach
% Gfdff = c2d(Tfc/Rfc,Ts,'zoh')
% Gfdfb = c2d(Sfc/Rfc,Ts,'zoh')
% Ifd = c2d(Ifc/s,Ts,'zoh')
% tustin approach
Gfdff = minreal(((2*tf1 + Ts*tf0)*z + Ts*tf0-2*tf1)/((Ts*r0 + 2)*z + Ts*r0-2))
Gfdfb = minreal(((Ts*sf0)*z + Ts*sf0)/((Ts*r0 + 2)*z + Ts*r0 - 2))
%Ifd = minreal(((Ifc*Ts)*z + Ifc*Ts)/(2*z - 2))
Ifd = minreal((If*Ts*z)/(z-1))
clear sf0 tf1 tf0

% Clears up the workspace
clear r0 s0 s1 w1 w2 w3 t0 t1 t2 g0 g1
print_modelyze_control(Sc/Rc,Tc/Rc);
print_control_struct( Gfdff, Gfdfb, Ifd, 1 )
print_control(1,umax,imax,K,n,R,Gfdff,Gfdfb,Ifd);
fprintf('\nCascaded position control PID-parameters:\n');
fprintf('P = %f, I = %f, D = %f\n',cpP,cpI,cpD);