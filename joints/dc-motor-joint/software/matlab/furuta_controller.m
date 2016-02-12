% Linearized control algorithm for the furuta pendulum
clear all; close all; clc
% params
g = 9.81;
L1 = 0.4735; l1 = 0.103611377105; m1 = 1.19468; J1zz = 0.0808111699502;
L2 = 0.433; l2 = 0.192564656882; m2 = 0.20168; J2xx = 1.476e-05;
J2yy = 0.00392556477687; J2zz = 0.00392556477687;
b1 = 0.1; b2 = 0.1; % friction
w_vel = 50;
% Simplified inertias
J1 = J1zz + m1*l1*l1;
J2 = max(J2yy,J2zz) + m2*l2*l2;
J0 = J1 + m2*L1*L1;

% State space model linearized around unstable position
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
A = [0 0 1 0; 0 0 0 1; A_31 A_32 A_33 A_34; A_41 A_42 A_43 A_44];
B = [0;0;B_31;B_32];
clear A_* B_* p
C = [0 1 0 0; 1 0 0 0]; % th2, th1 output
D = [0;0];
states = {'th1' 'th2' 'th1_dot' 'th2_dot'};
inputs = {'tau1'};
outputs = {'th2'; 'th1'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

% Control
R = 1; Q = C'*C; 
Q(1,1) = 1;    % th2 weight
Q(2,2) = 0.1;   % th1 weight
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
t = 0:0.01:4; u = 0.0*Nr*ones(size(t)); th2_init = 0.2;
y_sim = lsim(sys_ss_ctrl,u,t,[0,th2_init,0,0]);

[AX,H1,H2] = plotyy(t,y_sim(:,1),t,y_sim(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','th2')
set(get(AX(2),'Ylabel'),'String','th1')
title('Step Response with LQR Control')

% Some printouts
fprintf('tau_max = %f\n',max(abs(y_sim(:,3)))); % ~3.5 max
fprintf('Recommended Fs: %0.1f - %0.1f Hz\n',...
    (10*max(abs([Ec;pole(sys_ss)])))/2/pi,(30*max(abs([Ec;pole(sys_ss)])))/2/pi );
disp(' ');
sout = '// states = ['; i_max = length(states);
for i = 1:i_max
    if i < i_max
        sout = strcat(sout,sprintf('%s, ',states{i}));
    else
        sout = strcat(sout,sprintf('%s]\n',states{i}));
    end
end
disp(sout);
sout = 'static float L[] = {';
for i = 1:i_max
    if i < i_max
        sout = strcat(sout,sprintf('%0.4f, ',L(i)));
    else
        sout = strcat(sout,sprintf('%0.4f};',L(i)));
    end
end
disp(sout);
fprintf('static float Nr = %0.4f;\n\n',Nr);
clear sout i_max i

