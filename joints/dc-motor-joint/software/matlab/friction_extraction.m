function [k_lin,fit_lin,w,f,u] = friction_extraction(D,mtr,verbose)
% Extracts the friction from a stair plot
% input:
%   D: cell array where each cell is a log file struct
%   mtr: struct containing motor data
%   optional verbose level, if > 0 will display plots
%
% output:
%   constants and functions for calculating friction
%   working data for plotting

if nargin < 3
    verbose = 0;
end

if verbose > 0
    scrz = get(0,'ScreenSize');
    FIGURE_X = 600; FIGURE_Y = 300;
end

if verbose > 0
    figure, set(gcf,'Position',[scrz(3)*1/2-FIGURE_X/2, scrz(4)*1/2-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on
    cm = prism(length(D));
    x_max = 0;
    for i = 1:length(D)
        plot(D{i}.time,D{i}.vel,'Color',cm(i,:));
        if round(D{i}.time(end)) > x_max
            x_max = round(D{i}.time(end));
        end
    end
    clear i cm
    set(gca,'XTick',0:2:max(get(gca,'XTick')),'XLim',[0 x_max]); grid on
    xlabel('Time [sec]'); ylabel('\omega [rad/s]');
    title('Input to friction extraction');
end

% Finds w_max
ind = 1;
u = []; w = []; f = [];
fric_calc = @(u_in,w_max) 1/mtr.R*(mtr.eta*mtr.n*mtr.K * u_in - mtr.eta*mtr.n^2*mtr.K^2 * w_max);

for i = 1:length(D)
    iv = 1;
    while iv < length(D{i}.volt)

        uin = D{i}.volt(iv);
        is = D{i}.volt == uin;
        is(1:(iv-1)) = 0;
        idiff = find(diff(is(iv:end)) == -1,1);
        is((iv+idiff):end) = 0;
        
        iv = iv + idiff;
        
        if sum(is) < 50
            continue; % Must be a constant voltage for a time
        end
        
        is(cumsum(is) <= sum(is)*1/3) = 0; % Removes transients
        
        w_max = mean(D{i}.vel(is));
        if w_max < 0
            continue;
        end
        if (std(D{i}.vel(is))) > 0.05
            continue;
        end
%         if uin > 20
%             continue;
%         end
        
        %fprintf('@t = %f, w_max = %f, std: %f\n',min(D{i}.time(is)),w_max,std(D{i}.vel(is)));
        
        u(ind) = uin;
        w(ind) = w_max;
        %f(ind) = 1/mtr.R*(mtr.eta*mtr.n*mtr.K * uin - mtr.eta*mtr.n^2*mtr.K^2 * w(ind));
        f(ind) = fric_calc(u(ind),w(ind));

        ind = ind + 1;
    end
end
clear ind uin is iv

if verbose > 0
    figure, set(gcf,'Position',[scrz(3)*1/2-FIGURE_X/2, scrz(4)*1/2-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on, grid on
    plot(u,w,'.k');
    xlabel('u [V]'); ylabel('\omega [rad/s]');
    title('The angular speed w.r.t the voltage');

    set(gca,'YLim',[0 max(get(gca,'YLim'))]);
end


if verbose > 0
    figure, set(gcf,'Position',[scrz(3)*1/2-FIGURE_X/2, scrz(4)*1/2-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on, grid on
    plot(w,f,'.k');
    xlabel('\omega [rad/s]'); ylabel('T_f [Nm]');
    title('Experimental friction torque w.r.t. rotational speed');

    set(gca,'YLim',[0 max(get(gca,'YLim'))]);
end


% Least squares to find the static + dynamic friction combination that
% results in this friction

% We don't want the ones with small w to influence to much as these are in
% a nonlinear area
i_inc = w > 0.1;
wfit = w(i_inc);
ffit = f(i_inc);

% First order
A = [ones(length(wfit),1),wfit']; b = ffit';
x = A'*A\A'*b;
k_lin1 = x;
fit_lin1 = @(k,w) sign(w+1e-99)*k(1) + k(2)*w;


% Second order
A = [ones(length(wfit),1),wfit',wfit'.^2]; b = ffit';
x = A'*A\A'*b;
k_lin2 = x;
fit_lin2 = @(k,w) sign(w+1e-99)*k(1) + k(2)*w + sign(w)*k(3).*w.^2;
clear A b x i_inc wfit ffit
k_lin = k_lin2; fit_lin = fit_lin2; % Output 1:st or 2:nd order

% This is to similiar to above data
% % Least squares to fit an exponential curve to the data
% %fit_exp = @(k,w) sign(w)*k(2).*(1 - exp(-k(1)*abs(w))) + k(3)*w + sign(w)*k(4).*w.^2 ;  % Fitting function
% fit_exp = @(k,w) sign(w)*k(2).*(1 - exp(-k(1)*abs(w))) + k(3)*w;  % Fitting function
% %k_exp = lsqcurvefit(fit_exp,[10 0.1 0.1 0.1],w,f);
% k_exp = [50; k_lin];

if verbose > 1
    % Plots these results
    w_plot = linspace(0,max(w),1000);

    p(1) = plot(w_plot,fit_lin1(k_lin1,w_plot),'-b');
    p(2) = plot(w_plot,fit_lin2(k_lin2,w_plot),'-r');
    %plot(w_plot,fit_exp([k_lin(2) k_exp(2) k_lin(1)],w_plot),'--r');
    title('Estimated friction torque w.r.t. rotational speed');
    legend(p,'1:st order fit','2:nd order fit');
end

if verbose > 2
    
    FIGURE_X = 400; % Chubby plots
    
    % Additionally plots pure dynamic friction!
    i_inc = u < 24;
    wfit = w(i_inc);
    ffit = f(i_inc);
    ufit = u(i_inc);
    

    % Fits a function to the u vs w data
    %A = [ufit',ufit'.^2]; b = wfit';
    A = [ufit']; b = wfit';
    x = A'*A\A'*b;
    
    % plots
    k_uw = x;
    u2w = @(k,u) k(1)*u;
    %u2w = @(k,u) k(1)*u + sign(u)*k(2).*u.^2; 
    u_plot = linspace(0,24,length(w_plot));
    figure, set(gcf,'Position',[scrz(3)*1/2-FIGURE_X, scrz(4)*1/3-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on, grid on
    plot(ufit,wfit,'.k');
    plot(u_plot,u2w(k_uw,u_plot),'-r');
    title('Input voltage vs angular velocity');
    xlabel('u [V]'); ylabel('\omega [rad/s]');
    
    figure, set(gcf,'Position',[scrz(3)*1/2, scrz(4)*1/3-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on, grid on
    plot(wfit,ffit,'.k');
    plot(w_plot,fric_calc(u_plot,u2w(k_uw,u_plot)),'-r');

    % calculate the k's
    %A = [w_plot',w_plot'.^2]; b = fric_calc(u_plot,u2w(k_uw,u_plot))';
    A = [w_plot']; b = fric_calc(u_plot,u2w(k_uw,u_plot))';
    x = A'*A\A'*b;
    k_dyn = x;
    %fit_dyn = @(k,w) k(1)*w + sign(w)*k(2).*w.^2;
    title('Rotational speed vs friction torque');
    xlabel('\omega [rad/s]'); ylabel('T_f [Nm]');
    
%     A = [wfit',wfit'.^2]; b = ffit';
%     x = A'*A\A'*b;
%     k_dyn = x;
%     fit_dyn = @(k,w) k(1)*w + sign(w)*k(2).*w.^2;
%     figure, set(gcf,'Position',[scrz(3)*1/3-FIGURE_X/2, scrz(4)*1/3-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on, grid on
%     plot(wfit,ffit,'.k');    
%     plot(w_plot,fit_dyn(k_dyn,w_plot),'-g');
%     xlabel('\omega [rad/s]'); ylabel('T_f [Nm]');
%     title(sprintf('Dynamic friction: constants %f, %f',k_dyn(1),k_dyn(2)));

%fprintf('k_quadratic = [%f, %f]\n',k_dyn(1),k_dyn(2));
fprintf('k_lin = %f\n',k_dyn(1));
end

end



