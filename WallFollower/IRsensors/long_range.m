% Distance
d = [0.1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 15, 20, 30, 40, 50, 60, 70, 80];

% Analog value
a = [4, 140, 145, 220, 335, 360, 450, 520, 530, 515, 490, 450, 400, 330, 230, 185, 160, 140, 125, 110];

figure(1); clf; hold on;
plot(d, a);
plot(d, a, 'ro');
title('Long Range IR Sensor');
xlabel('Distance [cm]'); ylabel('Analog value');
