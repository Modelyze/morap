function ydata = beam_friction_fit( c,tdata,tspan,wstart,J )
% function used for the lsqcurvefit to fit friction coefficients
% to the beam friction model
%
% Return ydata for all things xdata

simfun = @(t,w) beam_model(t,w,J,c);
[Tsim,Ysim] = ode45(simfun,tspan,wstart);

ydata = interp1(Tsim,Ysim,tdata,'spline');


end

