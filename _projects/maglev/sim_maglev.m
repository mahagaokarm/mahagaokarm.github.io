%sim_maglev.m

%% simulation of model

% operating point (25 mm)
is = 250*10^(-3);  % 250 mA
ys = 25*10^(-3);  % 25 mm
us = 3.39 * is;

%% simulation parameters
xs = [is, ys, 0];
tspan = [0 0.05];

% input 0V - position goes to infinity (free fall)
u = 0;
[t, x] = ode45(@maglev, tspan, xs, [], u);
% plot(t, x(:,1:2)*10^(3));
plot(t, x(:,2)*10^(3));
% legend({'current (mA)', 'position (mm)'});

% operating point
u = us;
[t, x] = ode45(@maglev, tspan, xs, [], u);
% plot(t, x(:,1:2)*10^(3));
hold on, plot(t, x(:,2)*10^(3));


% input 3V - position goes to zero
u = 5;
[t, x] = ode45(@maglev, tspan, xs, [], u);
% plot(t, x(:,1:2)*10^(3));
hold on, plot(t, x(:,2)*10^(3));

xlabel('time (seconds)');
ylabel('position (mm)');
legend('0 V', '0.8475 V', '5 V');
title('Free Response');
grid on;

%% linearization
xs = [is; ys; 0];
[A, B] = GetLinModFtxu(@maglev, [], xs, us);
C = [0 1 0];
D = 0;

[NUM, DEN] = ss2tf(A, B, C, D);
G = tf(NUM, DEN);
zpk(G);

% sisotool(G)
% sisotool(G,Cs,1,.001)

%% SISO tool
s = tf('s');
Cs = (-371)*(s+28)/(s+100);
% Cs = (-259.95)*(s+20)/(s+50);
%496.64
[num, den] = tfdata(Cs);
cnum = num{:};
cden = den{:};

%% Linear Model
tsim = 1;
xs = [0;0;0];
ref = 0.001;  % 1mm

sim('maglev_linear.slx')
figure, plot(t, position*10^3);
xlabel('time (seconds)');
ylabel('position (mm)');
title('Linear Model');
grid on;

% control effort
figure, plot(t, ce);
xlabel('time (seconds)');
ylabel('input voltage (V)');
title('Control Effort', 'Linear Model');
grid on;

%% Nonlinear Model

tsim = 1;
xs = [is, ys, 0];
ref = 0.025+0.001;  

sim('maglev_nonlinear.slx')
figure, plot(t, position*10^3);
xlabel('time (seconds)');
ylabel('position (mm)');
title('Nonlinear Model');
grid on;

figure, plot(t, ce);
xlabel('time (seconds)');
ylabel('input voltage (V)');
title('Control Effort', 'Nonlinear Model');
grid on;


%% hardware
% Csd = c2d(Cs, 0.002);
% [num_d, den_d] = tfdata(Csd);
% cnum_d = num_d{:};
% cden_d = den_d{:};