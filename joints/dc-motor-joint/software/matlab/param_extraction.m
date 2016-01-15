% Extract data from logging files in order to extract parameters
clear all; close all; clc

% Motor data
mtr = struct();
% DCX26L
mtr.R = 0.74 + 0.33 + 18; mtr.L = 0.129e-3; mtr.K = 0.0214;
mtr.n = 150; mtr.eta = 0.75;

Ds = read_log_file('log_files/Joint1 Random1 +R18.cap');
[k_lin,fit_lin,w,f,u] = friction_extraction(Ds,mtr,1);


%% inertia extraction 

%Dt = read_log_file('log_files/Clean Trans +R36.cap');
Dt = read_log_file('log_files/Joint1 Trans +R18.cap');
%Dt = read_log_file('log_files/Joint1 Trans +R18 +0.3m.cap');

mtr.R = 0.74 + 0.33 + 18;   % Different winding resistance

f_fun = @(w) fit_lin(k_lin,w); % Friction resistance function 

J_best = inertia_extraction(Dt,mtr,f_fun,1);







