% Checks how the friction force depends on the normal force applied
clear all; close all; clc

% Motor data
mtr = struct();
% DCX26L
mtr.R = 0.74 + 0.33 + 18; mtr.L = 0.129e-3; mtr.K = 0.0214;
mtr.n = 150; mtr.eta = 0.75;



% Plotting
w_plot = linspace(0,6);
scrz = get(0,'ScreenSize');
FIGURE_X = 600; FIGURE_Y = 400;
fig1 = figure; set(gcf,'Position',[scrz(3)*1/3-FIGURE_X/2, scrz(4)*1/2-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on
fig2 = figure; set(gcf,'Position',[scrz(3)*2/3-FIGURE_X/2, scrz(4)*1/2-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on
p = []; % plot handles

Ds = read_log_file('log_files/Joint1 Random1 +R18.cap');
[k_lin,fit_lin,w,f,u] = friction_extraction(Ds,mtr,0);
figure(fig1), p(end+1:end+2) = plot(w,f,'or',w_plot,fit_lin(k_lin,w_plot),'-r');
figure(fig2), plot(u,w,'.r');

Ds = read_log_file('log_files/Joint1 Random1 +R18 +m3044.cap');
[k_lin,fit_lin,w,f,u] = friction_extraction(Ds,mtr,0);
figure(fig1), p(end+1:end+2) = plot(w,f,'*b',w_plot,fit_lin(k_lin,w_plot),'--b');
figure(fig2), plot(u,w,'.b');

Ds = read_log_file('log_files/Joint1 Random1 +R18 +m3835.cap');
[k_lin,fit_lin,w,f,u] = friction_extraction(Ds,mtr,0);
figure(fig1), p(end+1:end+2) = plot(w,f,'.g',w_plot,fit_lin(k_lin,w_plot),'-.g');
figure(fig2), plot(u,w,'.g');

% write stuff about stuff plots
figure(fig1)
legend(p,'+0 kg data','fit','+3.044 kg data','fit','+3.835 kg data','fit','Location','SouthEast')
title('Different friction fits depending on the applied normal force')
xlabel('\omega [rad/s]'); ylabel('T_f [Nm]');