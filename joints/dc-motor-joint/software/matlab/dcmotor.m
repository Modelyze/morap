function dw = dcmotor( t,w,u,K,R,L,J,n,eta,f_fun )
% Used for simulating dcmotors
%
% Inputs:
% t,w - time and state [w i] (used for ode45 functions)
% u - input voltage
% K,R,L,J,n,eta - motor parameters
% f_fun - a function describing the friction as a function of angular
%   velocity
%
% Outputs:
% dw - derivative of w

% dw = zeros(size(w));
% dw(1) = 1/L * ( u - R*w(1) - n*K*w(2) );
% dw(2) = 1/J * ( eta*n*K*w(1) - f_fun(w(2)) );


dw = 1/J * ( eta*n*K*(u-K*n*w)/R - f_fun(w) );


end

