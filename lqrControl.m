clc; clear; close all;

% Define system parameters
m = 0.1;   % Mass of pendulum
M = 0.3;   % Mass of cart
l = 0.5;   % Length of the pendulum
d = 0.1;   % Damping coefficient

% Create an Inverted Pendulum on Cart object
Invp = InvPendOnCart(m, M, l, d);

% Linearize the system to obtain state-space matrices A and B
[A, B] = Invp.Linearization();

% Define the desired state (reference)
wr = [0 0 pi 0]';

% Define the initial state
x0 = [0.2; 0; 160 * (pi / 180); 0];

% Define the time span for simulation
tspan = [0 10];

% Define the Q and R matrices for LQR control
Q = eye(4);     % State cost matrix
R = 0.001;      % Control cost matrix

% Calculate the LQR gain matrix K
K = lqr(A, B, Q, R);

% Define the control input function u(x) based on LQR control
u = @(x) K * (wr - x);

% Define the system dynamics function
f = @(t, x) Invp.computeDynamics(x, u(x));

% Define the time step and time vector
dt=0.01;
ttime = tspan(1):dt:tspan(end);

% Simulate the system using ODE45
[T, X] = odeSolver(f, ttime, dt,x0 ,'Rungekutta4');

% Plot the results
figure(1);
% Animation loop to visualize the motion of the pendulum
for i = 1:20:length(T)
    Invp.motionPlot(X(i, 1), X(i, 3));
    pause(0.1);
    if i~=length(T)
      clf;
    end 
    title('Lqr Controller For linearized System')
 end

% Plot the state variables over time
figure(2)
plot(T, X(:, 1), 'LineWidth', 2)
hold on
plot(T, X(:, 2), 'LineWidth', 2)
hold on
plot(T, X(:, 3), 'LineWidth', 2)
hold on
plot(T, X(:, 4), 'LineWidth', 2)
legend('x', 'v', '\theta', '\omega')
grid on
title('States')

