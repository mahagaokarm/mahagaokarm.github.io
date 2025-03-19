% maglev.m    11/26

function xdot = maglev(t, x, u)
xdot = zeros(3, 1);

% parameters
Rm = 3.39;      % ohms
L = 15 * 10^(-3);  % inductance
m = 0.003;  % weight of magnet (kg)
g = 9.8;
i0 = 250*10^(-3);  % 250 mA
y0 = 25*10^(-3);  % 25 mm
k = m*(9.8)/((i0)/(y0)^2);

i = x(1);
y = x(2);

xdot(1, 1) = (u - Rm*i)/L;
xdot(2, 1) = x(3);
xdot(3, 1) = (m*g - k*i/y^2)/m;
