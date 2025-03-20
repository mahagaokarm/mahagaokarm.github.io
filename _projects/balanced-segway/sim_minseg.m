% Meenakshi Mahagaokar  11/20/2024    Lab 6
% sim_minseg.m


%% MODEL AND SIMULATE SYSTEM
u = 0;
xs = [0;0;210/180*pi;0];
[t , x] = ode45(@minseg, [0 10], xs, [], u);

figure, plot(t, x(:, 1)*180/pi);
xlabel('time (s)');
ylabel('x (m)');
title('Wheel Position - no input voltage');
% title('Wheel Position - no input torque');

figure, plot(t, x(:, 3)*180/pi);
xlabel('time (s)');
% ylabel('alpha (radians)');
ylabel('alpha (degrees)')
title('Pendulum Angle - no input voltage');
% title('Pendulum Angle - no input torque');

%% LINEARIZE ABOUT OPERATING POINT
u = 0;
xs = [0;0;0;0];
[A, B] = GetLinModFtxu(@minseg, [], xs, u);

% LQR
R = 1;
Q = 100 * [1.2 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
KLQR = lqr(A, B, Q, R);
Rw = 0.0216;   % radius of wheel (m)


%% SIMULINK MODELS
%% LINEAR MODEL

tsim = 3;

ref = [0;0;0;0];        %for recovery
xs = [0;0;10/180*pi;0]; 

% ref = [0;0;30*pi/180;0];  %for step input
% xs = [0;0;0;0];

sim('minseg_linear.slx')
figure, plot(t, output(:,3));
% hold on, plot(t, output(:,3)*180/pi);
xlabel('time (seconds)');
ylabel('alpha (degrees)');
title('Pendulum Angle', 'Linear Model');
grid on;

hold on, plot(t, output(:, 1));
xlabel('time (s)');
ylabel('x (m)');
title('Wheel Position');

%% CONTROL EFFORT - LINEAR MODEL
% ce = reshape(u, 1, length(u));
% figure, plot(t, ce);
% xlabel('time (seconds)');
% ylabel('input voltage (V)');
% title('Control Effort', 'Linear Model');
% grid on;


%% NONLINEAR MODEL

tsim = 3;

ref = [0;0;0;0];        %for recovery
xs = [0;0;10/180*pi;0];   % does not work with large initial alpha

% ref = [0;0;30*pi/180;0];  %for step input
% xs = [0;0;0;0];

sim('minseg_nonlinear.slx')
out = reshape(output, 4, length(output));
figure, plot(t, out(3,:));
% hold on, plot(t, output(:,3)*180/pi);
xlabel('time (seconds)');
ylabel('alpha (degrees)');
title('Pendulum Angle', 'Nonlinear Model');
grid on;

hold on, plot(t, out(1,:));
xlabel('time (s)');
ylabel('x (m)');
title('Wheel Position');

%% CONTROL EFFORT - NONLINEAR MODEL
% ce = reshape(u, 1, length(u));
% figure, plot(t, ce);
% xlabel('time (seconds)');
% ylabel('input voltage (V)');
% title('Control Effort', 'Nonlinear Model');
% grid on;
