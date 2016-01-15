% Does a root locus for the load inertia
% requires the script DCmotorParams.m to be run
%
% If some dimension mismatch error occurs, just run it again
clc; close all;

Jv = linspace(0.5*J_design,1.5*J_design,21);
ps = zeros(length(Jv),length(pole(Gc)));
zs = zeros(length(Jv),length(zero(Gc)));
i = 1; idesign = 0;
for J = Jv
    Gs = minreal(eta*n*K/(R*J*s^2 + (eta*n*n*K*K+R*c)*s));
    [NUM,DEN] = tfdata(Gs);
    g0 = NUM{1}(3); g1 = DEN{1}(2);
    Ac = s^2 + g1*s; Bc = g0 + 0*s;
    Gc = minreal(Bc*Tc/(Ac*Rc + Bc*Sc));
    pls = pole(Gc)';
    zrs = zero(Gc)';
    if length(pls) < size(ps,2)
        pls = [pls, 1e6*ones(1,size(ps,2) - length(pls))];
    end
    if length(zrs) < size(zs,2)
        zrs = [zrs, 1e6*ones(1,size(zs,2) - length(zrs))];
    end
    ps(i,:) = pls;
    zs(i,:) = zrs;    
    if J == J_design
        idesign = i;
    end
    i = i + 1;
end
clear NUM DEN i pls zrs

% plots root locus
scrz = get(0,'ScreenSize');
FIGURE_X = 600; FIGURE_Y = 400;
fig = figure;
hold on;
set(fig,'Position',...
    [scrz(3)/2-FIGURE_X/2, scrz(4)/2-FIGURE_Y/2, FIGURE_X, FIGURE_Y]);
h = [];
for i = 1:size(ps,2)
    h(i) = plot(real(ps(ps(:,i) ~= 1e6,i)),imag(ps(ps(:,i) ~= 1e6,i)),'.b');
end
h(end+1) = plot(real(ps(1,:)),imag(ps(1,:)),'xr','MarkerSize',10);
h(end+1) = plot(real(ps(end,:)),imag(ps(end,:)),'or','MarkerSize',10);

if idesign == 0
    legend(h([end-1,end]),'J < J_{design}','J > J_{design}');
else
    h(end+1) = plot(real(ps(idesign,ps(idesign,:) ~= 1e6)),...
        imag(ps(idesign,ps(idesign,:) ~= 1e6)),'*r','MarkerSize',10);
    legend(h([end-2,end-1,end]),'J < J_{design}','J > J_{design}','J = J_{design}');
end

title('root locus with different load inertias');
xlabel('real'); ylabel('imag');

clear i h FIGURE_X FIGURE_Y fig scrz idesign



