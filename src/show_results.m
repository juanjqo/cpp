clear all
close all
clc

time_gp = readmatrix('time_gp.csv')';
time_ne = readmatrix('time_ne.csv')';

h1 = figure;
set(h1, 'DefaultTextFontSize', 10);
set(h1, 'DefaultAxesFontSize', 10); % [pt]
set(h1, 'DefaultAxesFontName', 'mwa_cmr10');
set(h1, 'DefaultTextFontName', 'mwa_cmr10');
set(h1, 'Units', 'centimeters');
pos = get(h1, 'Position');
pos(3) = 2*20; % Select the width of the figure in [cm] 17
pos(4) = 2*10; % Select the height of the figure in [cm] 6
set(h1, 'Position', pos);
set(h1, 'PaperType', 'a4letter');
set(h1,'PaperPositionMode','auto')
set(h1, 'Renderer', 'Painters');
a = 2;
b = 4;
w = 4;
fontsize = 20;


plot(1e3*time_gp(1,:),'b', 'LineWidth',w);
hold on
plot(1e3*mean(time_gp)*ones(1, length(time_gp)),'c', 'LineWidth',2);
hold on
plot(1e3*time_ne(1,:),':r', 'LineWidth',3);
hold on
plot(1e3*mean(time_ne)*ones(1, length(time_gp)),'m', 'LineWidth',2);
hold on
set(gca, 'FontSize', fontsize );
fig = gcf;
fig.Color = [1 1 1];
box('off');
ylabel({'Time','(ms)'})
xlabel('trials')
%title('$\tau$', 'Interpreter','latex', 'Rotation', 0)

legend('GPLC','GPLC-mean','NE','NE-mean')
