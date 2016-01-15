function [ J_best ] = inertia_extraction( D,mtr,f_fun,verbose )
% Calculates the inertia that bests fits the transient behaviors
% for the log files defined by D
%
% input:
%   D: cell array where each cell is a log file struct
%   mtr: struct containing motor data
%   f_fun: function defining the friction as a function of speed
%   optional verbose level, if > 0 will display plots
%
% output:
%   i_best: inertia that best fits with everything

% Function used with the lsqcurvefit function
function wdata = transient_fit_func(J,tdata,tspan,utest)
    f_ode45 = @(t,w) dcmotor(t,w,utest,mtr.K,mtr.R,mtr.L,J,mtr.n,mtr.eta,f_fun);
    [Tsim,Ysim] = ode45(f_ode45,tspan,0);
    wdata = interp1(Tsim,Ysim,tdata,'spline');
end


if nargin < 3
    verbose = 0;
end

if verbose > 0
    scrz = get(0,'ScreenSize');
end

Jv = zeros(length(D),1);
uv = zeros(length(D),1);
options = optimoptions('lsqcurvefit','Display','off');
for itest = 1:length(D)
    % Find simulation times
    utest = max(D{itest}.volt);
    i_start = find(D{itest}.volt == utest,1,'first');
%     i_end = find(D{itest}.volt == utest,1,'last');
%     t_sim = [D{itest}.time(i_start) D{itest}.time( (i_start + ceil(0.5*(i_end-i_start))) ) ];  
    i_end = i_start + ceil(0.25*(find(D{itest}.volt == utest,1,'last') - i_start));
    t_sim = [D{itest}.time(i_start) D{itest}.time(i_end)];
    
    
    lsq_fun = @(J,tdata) transient_fit_func(J,tdata,t_sim,utest);
    
    Jv(itest) = lsqcurvefit(lsq_fun,0.01,D{itest}.time(i_start:i_end),D{itest}.vel(i_start:i_end),0,[],options);
    uv(itest) = utest;
    fprintf('%d/%d\n',itest,length(D));
end % of d in D

% Calculate best while removing outliers
i_include = abs(Jv-mean(Jv)) < 1.25*std(Jv);
J_best = mean(Jv(i_include));

if verbose > 0
    FIGURE_X = 800;
    FIGURE_Y = 400;
    figure, set(gcf,'Position',[scrz(3)/2-FIGURE_X/2, 120+FIGURE_Y, FIGURE_X, FIGURE_Y]), hold on
    cm = hsv(length(D));
    lgnd_cell = cell(1,2*length(D));
    include_string = {' (ignored)',''};
    [~,i_order] = sort(uv,1,'descend');
    it = 1;
    for i = i_order'
        % Find simulation times
        utest = uv(i); %max(D{i}.volt);
        i_start = find(D{i}.volt == utest,1,'first');
        i_end = i_start + ceil(1.0*(find(D{i}.volt == utest,1,'last') - i_start));
        t_sim = [0 D{i}.time(i_end)-D{i}.time(i_start)];
        f_solv = @(t,w) dcmotor(t,w,utest,mtr.K,mtr.R,mtr.L,Jv(i),mtr.n,mtr.eta,f_fun);
        [T,Y] = ode45(f_solv,t_sim,0);

        plot(D{i}.time(i_start:i_end) - D{i}.time(i_start),D{i}.vel(i_start:i_end),'Color',cm(it,:));
        plot(T,Y,'Color',[0 0 0],'LineStyle','--');
        lgnd_cell{2*it-1} = sprintf('u = %0.1fV',utest);
        lgnd_cell{2*it} = sprintf('J = %f%s',Jv(i),include_string{i_include(i)+1});
        it = it + 1;
    end        
    legend(lgnd_cell);
    a = axis;
    axis([0 1 0 a(4)])
    title(sprintf('Comparison with experiment vs simulation: J = %f',J_best));
    xlabel('Time [sec]'); ylabel('\omega [rad/s]');
end

end

