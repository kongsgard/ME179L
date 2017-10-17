%% Sensor Calibration and Curve Fit
% Sondre Kongsgard and James Fanchiang
% 10/12/2017
%% Short Range IR Sensor Data
%
clear all;
clc;
% Distance
d = [ 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30];

% Analog value
a = [555, 460, 400, 300, 270, 250, 210, 155, 100, 85, 55];

figure(1); clf; hold on;
plot(d, a, 'ko');
title('Short Range IR Sensor');
xlabel('Distance [cm]'); ylabel('Analog value');

f1 = fit(d',a','exp1');
h = plot(f1,d,a)
legend( h, 'Analog Output vs. Distance', 'Fitted Curve', 'Location', 'NorthEast' );
xlabel('Distance [cm]');
ylabel('Analog value');
hold off;
f1
%%  Long Range IR Sensor Data
clear all;
clc;
% Distance
d = [10, 12, 15, 20, 30, 40, 50, 60, 70, 80];

% Analog value
a = [490, 450, 400, 330, 230, 185, 160, 140, 125, 110];

figure(2); clf; hold on;
plot(d, a, 'bo');
title('Long Range IR Sensor');

f2 = fit(d',a','exp1');
plot(f2,d,a)
h = plot(f2,d,a)
legend( h, 'Analog Output vs. Distance', 'Fitted Curve', 'Location', 'NorthEast' );
xlabel('Distance [cm]');
ylabel('Analog value');
hold off;
f2



