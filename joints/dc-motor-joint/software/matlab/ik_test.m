function ik_test
% Tests inverse kinematics for a scara type robotic arm
clc; close all;

function [a1,a2] = ik_calc(x,y,L1,L2)
    l = (x.^2 + y.^2).^0.5;
    if size(l,2) > size(l,1), l = l'; x = x'; y = y'; end
    a1 = atan2d(y,x) - acosd((L2^2-l.^2-L1^2)./(-2*l*L1));
    a2 = 180 - acosd((l.^2-L1^2-L2^2)/(-2*L1*L2));
    
    % Simplifies impossible angles, happens when the term in acos becomes 
    % > 1.
    imp = logical(sum(logical(imag([a1 a2])),2));
    a1(imp) = atan2d(y(imp),x(imp));
    
    a2_replace = zeros(sum(imp==1),1);
    smlr = l(imp) < (L1-L2);
    a2_replace(smlr) = 180;
    a2(imp) = a2_replace;
    
    % Debug info
    %[(L2^2-l.^2-L1^2)./(-2*l*L1) (l.^2-L1^2-L2^2)/(-2*L1*L2) imp l]
    
end

% Arm lengths
L1 = 0.257; L2 = 0.380;


% Sets up main figure
fig = figure(1);
scrz = get(0,'ScreenSize');
FIGURE_X = 0.75*min(scrz(3),scrz(4)); FIGURE_Y = FIGURE_X;
set(fig,'Position',...
    [scrz(3)/2-FIGURE_X/2 scrz(4)/2-FIGURE_Y/2 FIGURE_X FIGURE_Y]);
clear FIGURE_X FIGURE_Y scrz

axis equal
axis manual
axis([-1.2*(L1+L2) 1.2*(L1+L2) -1.2*(L1+L2) 1.2*(L1+L2)]);
hold on

% plot handles
p_paint = plot(NaN,NaN);
set(p_paint,'Color','black','LineWidth',2,'LineStyle','-');
p = [];
p(1) = plot(NaN,NaN);
set(p(1),'Color','blue','LineWidth',4,'LineStyle','-');
p(2) = plot(NaN,NaN);
set(p(2),'Color','red','LineWidth',4,'LineStyle','-');
pend = plot(NaN,NaN);
set(pend,'Color','black','LineStyle','none','Marker','x','MarkerSize',6);


% Time step when plotting
Ts = 1/30;

% % Angles
% y_wanted = 0.1; %linspace(-0.5,0.5,100);
% x_wanted = 0.2; %linspace(-0.5,0.75,100);
% [ang_1,ang_2] = ik_calc(x_wanted,y_wanted,L1,L2);

v = 0.5; %m/s
R = (0.25*(L1+L2)); x_0 = (0.66*(L1+L2)); y_0 = 0;
tv = 0:Ts:19;
x_wanted = x_0 + R*cos(v/R*tv);
y_wanted = y_0 + R*sin(v/R*tv);
[ang_1,ang_2] = ik_calc(x_wanted,y_wanted,L1,L2);

% data = importdata('minicom.cap');
% tv = 0:Ts:data(end,1);
% ang_1 = interp1(data(:,1),data(:,2),tv);
% ang_2 = interp1(data(:,1),data(:,3),tv);

%[ang_1,ang_2] = ik_calc(0.43 + 0.16,0.16,L1,L2);

paint_vec = zeros(length(ang_1),2);
paint_pos = 1;

a1_old = ang_1(1); a2_old = ang_2(1);
for i = 1:length(ang_1)
    if imag(ang_1(i)) | imag(ang_2(i))
        fprintf('imag detected @ x = %0.2f, y = %0.2f\n',x_wanted(i),y_wanted(i));
        %keyboard;
    end
    fprintf('a1 = %0.2f, a2 = %0.2f\n',ang_1(i),ang_2(i));
    x1 = L1*cosd(ang_1(i)); y1 = L1*sind(ang_1(i));
    x2 = x1 + L2*cosd(ang_2(i)+ang_1(i)); y2 = y1 + L2*sind(ang_2(i)+ang_1(i));

    set(p(1),'XData',[0 x1],'YData',[0 y1]);
    set(p(2),'XData',[x1 x2],'YData',[y1 y2]);
    if exist('y_wanted','var')
        set(pend,'XData',x_wanted(i),'YData',y_wanted(i));
    end
    paint_vec(paint_pos,:) = [x2 y2];
    set(p_paint,'XData',paint_vec(1:paint_pos,1),'YData',paint_vec(1:paint_pos,2) );
    paint_pos = paint_pos + 1;
    
    title(sprintf('Time: %0.2f. w_1 = %0.2f rpm, w_2 = %0.2f rpm',i*Ts,(ang_1(i)-a1_old)/Ts/6,(ang_2(i)-a2_old)/Ts/6));
    a1_old = ang_1(i); a2_old = ang_2(i);
    %fprintf('x = %0.2f, y = %0.2f \n',x2,y2);
    pause(Ts);
end

disp('done');
end
