% Meenakshi Mahagaokar  11/20/2024    Lab 6
% minseg.m

function xdot = minseg(t, x, u)

xdot = zeros(4, 1);
xdot(1, 1) = x(2);
xdot(3, 1) = x(4);

% parameters
g = 9.81;
lp = 0.095;    % 95mm - segway
mp = 0.187;    % kg without batteries (doesn't include wheels)
% mp = 0.187 + 0.327;  % with batteries
mw = 0.036;    % wheels and axle
Rm = 5.2;      % ohms
Rw = 0.0216;   % radius of wheel (m)
Kb = 0.495;    % Vs/rad
Kt = 0.32;     % Nm/A
Iw = 1/2*mw*Rw^2;

% Tm = u;                                   % input -> torque
Tm = Kt/Rm*(u + Kb*x(4) + Kb*x(2)/Rw);  % input -> voltage 
alpha = x(3);
C = [mw 0 1 -1 0 0;
    0 0 0 0 1 1;
    -Iw/Rw 0 Rw  0 0 0;
    mp -mp*lp*cos(alpha) 0 1 0 0;
    0 mp*lp*sin(alpha) 0 0 -1 0;
    0 0 0 lp*cos(alpha) lp*sin(alpha) 0
    ];
b = [0;
    mw*g;
    Tm;
    -mp*lp*x(4)^2*sin(alpha);
    mp*g-mp*lp*x(4)^2*cos(alpha);
    -Tm    %originally +Tm not sure why this gives correct matrices
    ];
z = inv(C)*b;

xdot(2, 1) = z(1);
xdot(4, 1) = z(2);



