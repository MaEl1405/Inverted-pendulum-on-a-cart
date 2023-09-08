% Clear workspace, close figures, and clear command window
clc;
clear;
close all;

%% Parameters
% Define system parameters
m = 0.1;        % Mass of the pendulum
M = 0.3;        % Mass of the cart
L = 0.4;        % Length of the pendulum
b = 0.00008;    % Viscous damping
c = 0.7;        % Viscous friction
I = 0.0007;     % Moment of inertia of the pendulum
g = 9.81;       % Acceleration due to gravity

%% LQR Controller
% Create an inverted pendulum on a cart object
Invp = InvPendOnCart(m, M, L, c, b, I);
% Linearize the system to obtain state-space matrices A and B
[A, B] = Invp.Linearization();

% Define the Q and R matrices for LQR control
Q = eye(4);      % State cost matrix
R = 0.035;       % Control cost matrix
% Calculate the LQR gain matrix K
K_lqr = lqr(A, B, Q, R);   

%% Initialization
u  = 0;                        % Initialize control input
% Random initial angle within [-pi, pi]
random_theta = (2 * pi) * rand() - pi;
wr = [0; 0; pi; 0];            % Reference state  (Desired States)
x0 = [0; 0; random_theta; 0];  % Replace with your actual initial state
t  = 0;                        % Initial time
ii = 0;                        % Iteration counter
dt = 0.001;                    % Time step
Tf = 10;                       % Final simulation time
X  = zeros(round(Tf/dt)+1,4);  % states vector

%% Simulation Loop
for k = 0:dt:Tf
    % Define system dynamics function
    f = @(t, x) Invp.computeDynamics(x, u);
    ii = ii + 1;
    
    % Perform RK4 integration to get the new state
    new_state = RK4(f, x0, t, dt);
    
    % Update time and state for the next iteration
    t = t + dt;
    x0 = new_state;
    theta = new_state(3);
    thdot = new_state(4);
    xdot = new_state(2);
   
    % Store the state for further analysis or visualization
    X(ii,:) = new_state';
    
    %% Swing-up Control
    % Calculate the total energy of the pendulum
    Et = m * g * L * (1 - cos(theta)) + (1/2) * (I + m * L^2) * (thdot^2);
    Ed = 2 * m * g * L;  % Desired states energy for swing-up control

    % Energy-shaping based swing-up control input
    k_swing =2.5;   
    swing_a = k_swing * g * (Et - Ed) * sign(thdot * cos(theta));
    u_swing = (M + m) * swing_a + 0.063 * xdot - m * L * (thdot^2) * sin(theta) ...
        - m * L * (cos(theta)) * ((b * thdot + m * L * swing_a * cos(theta) + m * g * L * sin(theta)) / (I + m * L^2));

    %% Switching the control input based on the value of theta
    if (abs(abs(wr(3)) - abs(theta))) * (180/pi) <= 30
       u = -K_lqr * (new_state - wr);  % LQR control
    else
       u = u_swing;                    % Swing-up controller
       U_swing(ii,1) = u;              % Store the swing-up control input
    end
   
end

%% Visualization
% Motion animation
% Initialize VideoWriter
% videoFile = VideoWriter('Swing_up_InvertedPP6.mp4', 'MPEG-4');
% open(videoFile);

% Loop for creating animation frames and saving to the video
figure(1)
for ii = 1:40:length(X)
    % Create the animation plot
    Invp.motionPlot(X(ii, 1), X(ii, 3));
    
    % Pause briefly to display the frame
    pause(0.01);
    if ii~=length(X)
        clf;
    end
    %%Capture the current frame
    %current_frame = getframe(gcf);
    %writeVideo(videoFile, current_frame);
    %close(gcf);
end

%%Close the VideoWriter object
%close(videoFile);

% U_swing plot if it exist
if exist('U_swing', 'var') == 1
    figure(2)
    plot(U_swing,'LineWidth',1.75,'Color','black')
    xlabel('Time Step')
    ylabel('Control Input')
    title('Swing-up Control Input Over Time')
    grid on
end

% State variables plot
figure(3)
plot(X, 'LineWidth', 1.75);
grid minor
legend('X', 'xdot', 'theta', 'theta_dot')
xlabel('Time Step')
ylabel('State Variable Value')
title('State Variables Over Time')
