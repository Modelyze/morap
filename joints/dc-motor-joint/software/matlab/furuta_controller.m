% Linearized control algorithm for the furuta pendulum
clear all; close all; clc
% params
g = 9.81;
L1 = 0.2735; l1 = 0.0539785034283; m1 = 1.11134; J1zz = 0.0539410345514;
L2 = 0.533; l2 = 0.242297308404; m2 = 0.24335; J2xx = 1.845e-05; 
J2yy = 0.00699995483974; J2zz = 0.00699995483974;

b1 = 0.005; b2 = 0.002; % friction

% Dc-motor equations
mtr = struct();
mtr.K = 0.0214; mtr.R = 0.74+0.33; mtr.L = 0.129e-3; 
mtr.n = 328509/2197; mtr.eta = 0.75;
% Stuff
w_vel = 50; % Pole of speed controller
% Simplified inertias
J1 = J1zz + m1*l1*l1;
J2 = mean([J2yy,J2zz]) + m2*l2*l2;
J0 = J1 + m2*L1*L1;

input = 'dc-motor';

% State space model linearized around unstable position with torque input
p = (J0*J2 - m2*m2*L1*L1*l2*l2);
A_31 = 0;
A_32 = g*m2*m2*l2*l2*L1/p;
A_33 = -b1*J2/p;
A_34 = -b2*m2*l2*L1/p;
A_41 = 0;
A_42 = g*m2*l2*J0/p;
A_43 = -b1*m2*l2*L1/p;
A_44 = -b2*J0/p;
B_31 = J2/p;
B_41 = m2*L1*l2/p;
B_32 = m2*L1*l2/p;
B_42 = J0/p;

% Staties [th1,th2,th1_dot,th2_dot]
if strcmpi(input,'torque')
    A = [0 0 1 0; 0 0 0 1; A_31 A_32 A_33 A_34; A_41 A_42 A_43 A_44];
    B = [0;0;B_31;B_41];
    inputs = {'tau1'};
elseif strcmpi(input,'dc-motor')
    A_33 = A_33 - B_31*mtr.K*mtr.K*mtr.n*mtr.n*mtr.eta/mtr.R;
    A_43 = A_43 - B_41*mtr.K*mtr.K*mtr.n*mtr.n*mtr.eta/mtr.R;
    A = [0 0 1 0; 0 0 0 1; A_31 A_32 A_33 A_34; A_41 A_42 A_43 A_44];
    B = [0;0; mtr.K*mtr.n*mtr.eta*B_31/mtr.R; mtr.K*mtr.n*mtr.eta*B_41/mtr.R];
    inputs = {'u'};
elseif strcmpi(input,'speed')
    A_41 = A_41 - A_31*B_41/B_31;
    A_42 = A_42 - A_32*B_41/B_31;
    A_43 = A_43 - A_33*B_41/B_31 - w_vel*B_41/B_31;
    A_44 = A_44 - A_34*B_41/B_31;
    B_41 = w_vel*B_41/B_31;
    A_31 = 0; A_32 = 0; A_33 = -w_vel;  A_34 = 0; B_31 = w_vel;
    A = [0 0 1 0; 0 0 0 1; A_31 A_32 A_33 A_34; A_41 A_42 A_43 A_44];
    B = [0;0;B_31;B_41];
    inputs = {'w_ref'};
end
clear A_* B_* p
C = [0 1 0 0; 1 0 0 0]; % th2, th1 output
D = [0;0];
states = {'th1' 'th2' 'th1_dot' 'th2_dot'};

outputs = {'th2'; 'th1'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%% Control
Q = C'*C;
if strcmpi(input,'torque')
    R = 1;
    Q(1,1) = 10;  % th2 weight
    Q(2,2) = 4;   % th1 weight
elseif strcmpi(input,'dc-motor')
    R = 1;        %   
    Q(1,1) = 1;   % th2 weight
    Q(2,2) = 1;   % th1 weight
elseif strcmpi(input,'speed')
    R = 10;        %   
    Q(1,1) = 1;   % th2 weight
    Q(2,2) = 1;   % th1 weight
end
    
[L,~,Ec] = lqr(A,B,Q,R); 
Ac = A - B*L; Bc = B; Cc = [C; -L]; Dc = [D; 1];
inputs = {'r'}; outputs{3} = 'u';
sys_ss_ctrl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% Ref scaling
Nr = 1; tfs = tf(sys_ss_ctrl); [i_max,~] = size(tfs);
for i = 1:i_max
    if strcmp(tfs.outputname{i},'th1')
        Nr = tfs.den{i}(end)/tfs.num{i}(end);
        sys_ss_ctrl.D(3) = Nr;
        break;
    end
end
clear i i_max tfs

% Simulation
t = 0:0.01:10; u = 0.0*Nr*ones(size(t)); th2_init = 0.2;
y_sim = lsim(sys_ss_ctrl,u,t,[0,th2_init,0,0]);

[AX,H1,H2] = plotyy(t,y_sim(:,1),t,y_sim(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','th2')
set(get(AX(2),'Ylabel'),'String','th1')
title('Step Response with LQR Control')

% Some printouts
if strcmpi(input,'torque')
    fprintf('tau_max = %f\n',max(abs(y_sim(:,3)))); % ~3.5 Nm max
elseif strcmpi(input,'dc-motor')
    fprintf('u_max = %f\n',max(abs(y_sim(:,3)))); % ~24 Nm max
elseif strcmpi(input,'speed')
    fprintf('w_ref_max = %f\n',max(abs(y_sim(:,3)))); % ~7 Nm max
end
fprintf('Recommended Fs: %0.1f - %0.1f Hz\n',...
    (10*max(abs([Ec;pole(sys_ss)])))/2/pi,(30*max(abs([Ec;pole(sys_ss)])))/2/pi );
disp(' ');
sc_out{1} = '// state feedback parameters: states = [';
sm_out{1} = '// state feedback parameters: states = ['; 
i_max = length(states);
for i = 1:i_max
    if i < i_max
        sc_out{1} = strcat(sc_out{1},sprintf('%s, ',states{i}));
        sm_out{1} = strcat(sm_out{1},sprintf('%s, ',states{i}));
    else
        sc_out{1} = strcat(sc_out{1},sprintf('%s]\n',states{i}));
        sm_out{1} = strcat(sm_out{1},sprintf('%s]\n',states{i}));
    end
end
sc_out{2} = 'const float L[] = {';
sm_out{2} = '';
for i = 1:i_max
    if i == 1
        sc_out{2} = strcat(sc_out{2},sprintf('%0.4f, ',L(i)));
        sm_out{2} = strcat(sm_out{2},sprintf('def L_%d = %0.4f; ',i,L(i)));
    elseif i < i_max
        sc_out{2} = strcat(sc_out{2},sprintf(' %0.4f, ',L(i)));
        sm_out{2} = strcat(sm_out{2},sprintf(' def L_%d = %0.4f; ',i,L(i)));
    else
        sc_out{2} = strcat(sc_out{2},sprintf(' %0.4f};',L(i)));
        sm_out{2} = strcat(sm_out{2},sprintf(' def L_%d = %0.4f; ',i,L(i)));
    end
end
sc_out{3} = sprintf('const float Nr = %0.4f;\n',Nr);
sm_out{3} = sprintf('def Nr = %0.4f;\n',Nr);
fprintf('C-code declaration of control variables\n');
for i = 1:length(sc_out), disp(sc_out{i}); end
fprintf('\nModelyze declaration of control variables\n');
for i = 1:length(sm_out), disp(sm_out{i}); end

clear sc_out i_max i

