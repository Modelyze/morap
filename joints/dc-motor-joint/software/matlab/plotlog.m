% Plots the content of a log file generated by the data logging application
clear all; close all; clc
%%
D = read_log_file('log_files/Endlink1 Friction1.cap');

%% Plots
scrz = get(0,'ScreenSize');
FIGURE_X = 600; FIGURE_Y = 400;

figure, set(gcf,'Position',[scrz(3)*1/2-FIGURE_X/2, scrz(4)*1/2-FIGURE_Y/2, FIGURE_X, FIGURE_Y]), hold on
cm = hsv(length(D)); ls = {'-','--','-.',':'};
lgnd_cell = {};
for i = 1:length(D)
    for ic = 1:size(D{i}.vel,2)
        plot(D{i}.time,D{i}.vel(:,ic),'Color',cm(i,:),'LineStyle',ls{ic});
        if ic == 1
            lgnd_cell{end+1} = D{i}.name;
        else
            lgnd_cell{end+1} = '';
        end
    end
end
legend(lgnd_cell);
xlabel('time [sec]'); ylabel('\omega [rad/s]');
title('Log files');
clear lgnd_cell