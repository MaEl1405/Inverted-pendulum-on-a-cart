clc; clear; close all;

%%  Define system parameters

m=0.1;
M=0.3;
L=0.5;
b=0.00008 ;
c= 0.7;
I=0.0007;

%% 
% Create an Inverted Pendulum on Cart object
Invp = InvPendOnCart(m,M,L,c,b,I);

% Linearize the system to obtain state-space matrices A and B
[A, B] = Invp.Linearization();

% Define the desired state (reference)
wr = [0 0 pi 0]';

% Define the initial state
x0 = [0; 0; 150 * (pi / 180); 0];

% Define the time span for simulation
tspan = [0 10];

% Define the Q and R matrices for LQR control
Q = eye(4);     % State cost matrix
R = 0.00035;      % Control cost matrix

% Calculate the LQR gain matrix K
K = lqr(A, B, Q, R);

% Define the control input function u(x) based on LQR control
u = @(x) K * (wr - x);

% Define the system dynamics function
f = @(t, x) Invp.computeDynamics(x, u(x));

% Define the time step and time vector
h=0.001;
ttime = tspan(1):h:tspan(end);

% Simulate the system using ODE45
[T, X] = odeSolver(f,ttime,h,x0,'Rungekutta4');

% Plot the results
%Animation loop to visualize the motion of the pendulum
for ii = 1:100:length(T)
    Invp.motionPlot(X(ii, 1), X(ii, 3));
    pause(0.01);
    if ii~=length(T)
      clf;
    end
    title('LQR Controller For linearized System')
    
 end

% Plot the state variables over time
figure(2)
plot(T, X(:, 1), 'LineWidth', 1.75)
hold on
plot(T, X(:, 2), 'LineWidth', 1.75)
hold on
plot(T, X(:, 3), 'LineWidth', 1.75)
hold on
plot(T, X(:, 4), 'LineWidth', 1.75)
legend('x', 'v', '\theta', '\omega')
grid on
title('States')

