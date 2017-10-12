% Distance
d = [0.1, 0.5, 1, 2, 3, 4, 5, 6, 7, 10, 15, 20, 25, 30];

% Analog value
a = [3, 300, 450, 500, 670, 640, 530, 450, 380, 260, 175, 110, 80, 65];

figure(1); clf; hold on;
plot(d, a);
plot(d, a, 'ro');
title('Short Range IR Sensor');
xlabel('Distance [cm]'); ylabel('Analog value');