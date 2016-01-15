% Extracts the friction from a free running beam with known inertia
% The input to this is a log file in which the user accelerates a sliding
% mass and lets it go so it stops only by friction.
%
% Everything serves the beam
clear all; close all; 
clc

D = read_log_file('log_files/Endlink1 Friction1.cap');

% Params
L = 0.2;
J = 0.4*L^3/12 + 0.4*L*(L/2 - 0.01)^2;

% Find the interesting descending transients
T = {};
for i = 1:length(D)
    
    t = 0; T_period = 1.5;
    while t < D{i}.time(end) - 1.1*T_period
        is = find(D{i}.time > t & D{i}.time < (t+T_period));
        % Glorious if-sats
        if(abs(mean(D{i}.vel(is(1:10)))) < 0.15 && abs(mean(D{i}.vel(is(end-9:end)))) < 0.15 && max(abs(D{i}.vel(is))) > 2)
            T{end+1} = struct();
            
            i_max = find(abs(D{i}.vel(is)) == max(abs(D{i}.vel(is))),1);
            T{end}.time = D{i}.time(is(i_max):is(end)) - D{i}.time(is(i_max));
            if abs(max(D{i}.vel(is))) > abs(min(D{i}.vel(is)))
                T{end}.vel = D{i}.vel(is(i_max):is(end));
            else
                T{end}.vel = -1*D{i}.vel(is(i_max):is(end));
            end
            
%             T{end}.time = D{i}.time(is);% - D{i}.time(is(1));
%             if abs(max(D{i}.vel(is))) > abs(min(D{i}.vel(is)))
%                 T{end}.vel = D{i}.vel(is);
%             else
%                 T{end}.vel = -1*D{i}.vel(is);
%             end
            
            t = t + T_period;
        end
        
        t = t + 0.1;
    end
        
end

%% Plots results from above
scrz = get(0,'ScreenSize');
FIGURE_X = 600; FIGURE_Y = 400;

figure, set(gcf,'Position',[scrz(3)*1/2-FIGURE_X, scrz(4)-FIGURE_Y-85, FIGURE_X, FIGURE_Y]), hold on
cm = hsv(length(T));
for i = 1:length(T)
    plot(T{i}.time,T{i}.vel,'Color',cm(i,:));
end


%% Attempts to find best k's using lsqcurvefit, both static + dynamic and only dynamic
kopt2 = zeros(length(T),2); kopt1 = zeros(length(T),1);
options = optimoptions('lsqcurvefit','Display','off');
for it = 1:length(T)
    lsq_fun = @(k,xdata) beam_friction_fit(k,xdata,[T{it}.time(1), T{it}.time(end)],T{it}.vel(1),J);
    kopt2(it,:) = lsqcurvefit(lsq_fun,[0.01 0.01],T{it}.time,T{it}.vel,[0 0],[],options);
    kopt1(it) = lsqcurvefit(lsq_fun,0.01,T{it}.time,T{it}.vel,0,[],options);
end

%% Plots results from above
if exist('c','var'), close all; end;
c1 = mean(kopt1);
c2 = mean(kopt2); % Which model to fit too

simfun1 = @(t,w) beam_model(t,w,J,c1);
simfun2 = @(t,w) beam_model(t,w,J,c2);

% Cool plots
per_plot = 3; % how many simulations per plot window
FIGURE_SMALL_X = floor((scrz(3)-100)/ceil(length(T)/per_plot));
posX = FIGURE_SMALL_X*[0:(ceil(length(T)/per_plot)-1)] + 100;
cm = hsv(per_plot);

ip = 1;
for it = 1:length(T)
    [Tsim1,Ysim1] = ode45(simfun1,[T{it}.time(1) T{it}.time(end)],T{it}.vel(1));
    [Tsim2,Ysim2] = ode45(simfun2,[T{it}.time(1) T{it}.time(end)],T{it}.vel(1));
    
    if mod(it-1,per_plot) == 0
        figure, set(gcf,'Position',[posX(ip), 10, FIGURE_SMALL_X, FIGURE_Y]), hold on
        xlabel('Time [sec]'); ylabel('\omega [rad/s]');
        title('Fitted friction transient (--) to data (-)');
        ip = ip + 1;
    end
    plot(T{it}.time,T{it}.vel,'Color',cm((mod(it-1,per_plot) + 1),:));
    plot(Tsim1,Ysim1,'Color',cm((mod(it-1,per_plot) + 1),:),'LineStyle','--');
    plot(Tsim2,Ysim2,'Color',cm((mod(it-1,per_plot) + 1),:),'LineStyle',':');
    
    
end

% plots friction
figure, set(gcf,'Position',[scrz(3)*1/2, scrz(4)-FIGURE_Y-85, FIGURE_X, FIGURE_Y]), hold on
w2f = @(w,c) sign(w+1e-99)*c(1) + c(2)*w;
w_plot = [linspace(-8,-1e-6,3) linspace(1e-6,8,3)];
plot(w_plot,w2f(w_plot,mean(kopt2)),'-b');
plot(w_plot,mean(kopt1)*w_plot,'-r');

legend('static + dynamic','only dynamic');
title(sprintf('k2 = [%s], k1 = %0.5f',num2str(mean(kopt2),3),mean(kopt1)));













