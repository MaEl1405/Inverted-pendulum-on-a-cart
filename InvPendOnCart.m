% InvPendOnCart - Represents an inverted pendulum On a Cart system.
%
%   This class encapsulates the properties and behavior of an inverted
%   pendulum system, including its physical parameters and dynamics
%   calculations.
%
%   Properties:
%       - m: Mass of the pendulum bob (kg)
%       - M: Mass of the cart (kg)
%       - L: Length of the pendulum (m)
%       - g: Gravitational acceleration (m/s^2)
%       - d: Damping coefficient for the cart (N*s/m)
%       
%
%   Methods:
%       - computeDynamics: Compute the system dynamics.
%
%   Example:
%       pendulum = InvPendOnCart(0.1,1.0,0.5,9.81,0.01,[0, 0, pi/4, 0]);
%
classdef InvPendOnCart
    properties
        m       % bob mass
        M       % Cart mass
        L       % Pendulum length
        d       % Damping Coefficient 
    end

    properties(Constant)
        g=-9.8; %gravitational acceleration

        % properties for animation
        W =0.32       %cart with;
        H = 0.1;    % Cart Height
        Wh = 0.05;   %wheel position
    end

    methods
        % Constructor method
        function obj = InvPendOnCart(mPend,mCart,length,damping)
            % InvPendOnCart constructor.
            %
            %   Creates an instance of the InvPendOnCart class with the 
            %   given parameters.
            %
            % Inputs:
            %   mPend - Mass of the pendulum bob (kg)
            %   mCart - Mass of the cart (kg)
            %   length - Length of the pendulum (m)
            %   damping - Damping coefficient for the cart (N*s/m)
            %
            obj.m = mPend;
            obj.M = mCart;
            obj.L = length;
            obj.d = damping;
        end %End of Constructor Method

        % Function to compute dynamics
        function dx = computeDynamics(obj,X,u)
            %computeDynamics - Compute the dynamics of the inverted
            %                  pendulum system.
            %
            % Inputs:
            %   X - State vector [x, x_dot, theta, theta_dot]
            %   t - Time vector
            %
            % Outputs:
            %   dx - Time derivative of the state vector
            %        [x_dot, x_ddot, theta_dot, theta_ddot]
            %

           
            % State space reprsentation
            Sx = sin(X(3));
            Cx = cos(X(3));
            D = obj.m * obj.L^2 * (obj.M + obj.m * (1 - Cx^2));

            dx(1,1) = X(2);
            dx(2,1) = (1/D)*(-(obj.m)^2*(obj.L)^2*(obj.g)*Cx*Sx +... 
                (obj.m)*(obj.L)^2*((obj.m)*(obj.L)*X(4)^2*Sx ...
                - (obj.d)*X(2))) + (obj.m)*(obj.L)*(obj.L)*(1/D)*u;
            dx(3,1) = X(4);
            dx(4,1) = (1/D)*((obj.m+obj.M)*(obj.m)*(obj.g)*(obj.L)*Sx ...
                - (obj.m)*(obj.L)*Cx*((obj.m)*(obj.L)*X(4)^2*Sx ... 
                 -(obj.d)*X(2))) - (obj.m)*(obj.L)*Cx*(1/D)*u;
        end % End of computeDynamics

        function [A, B] = Linearization(obj)
            % Linearization - Compute the linearized state-space 
            %           representation  of the inverted pendulum system.
            %
            % Inputs:
            %   obj - An object representing the inverted pendulum system
            %
            % Outputs:
            %   A - State transition matrix
            %   B - Input matrix
            %

            % Define symbolic variables for the state (x1, x2, x3, x4)
            % and control input (u)
            syms x1 x2 x3 x4 u real; % Define symbolic variables as real

            % Create a symbolic state vector X
            X = [x1 x2 x3 x4]';

            % Initialize the symbolic dynamics vector f
            f = zeros(4, 1, 'sym');

            % Compute sine and cosine of the angle theta
            Sx = sin(X(3));
            Cx = cos(X(3));

            % Calculate the denominator factor D
            D = obj.m * obj.L^2 * (obj.M + obj.m * (1 - Cx^2));

            % Define the dynamics equations f(1) to f(4)
            f(1) = X(2);
            f(2) = (1/D) * (-(obj.m)^2 * (obj.L)^2 * (obj.g) * Cx *...
              Sx + (obj.m) * (obj.L)^2 * ((obj.m) * (obj.L) * X(4)^2 ...
                * Sx - (obj.d) * X(2))) + (obj.m) * (obj.L) * (obj.L)...
                  * (1/D) * u;
            f(3) = X(4);
            f(4) = (1/D) * ((obj.m + obj.M) * (obj.m) * (obj.g) *...
                (obj.L) * Sx - (obj.m) * (obj.L) * Cx * ((obj.m) *...
                (obj.L) * X(4)^2 * Sx - (obj.d) * X(2))) - (obj.m) * ...
                (obj.L) * Cx * (1/D) * u;

            % Compute the Jacobian matrices A and B
            A1 = jacobian(f, X); % Compute the Jacobian matrix with respect to X
            B1 = jacobian(f, u); % Compute the Jacobian matrix with respect to u

            % Define equilibrium point values as symbolic variables
            x1_eq = 0;   % Equilibrium position
            x2_eq = 0;   % Equilibrium velocity
            x3_eq = pi;  % Equilibrium angle (in radians)
            x4_eq = 0;   % Equilibrium angular velocity
            u_eq = 0;    % Equilibrium control input

            % Substitute the equilibrium values into A1 and B1 and convert to double
            A = double(subs(A1, [x1, x2, x3, x4, u], [x1_eq, x2_eq, x3_eq, x4_eq, u_eq]));
            B = double(subs(B1, [x1, x2, x3, x4, u], [x1_eq, x2_eq, x3_eq, x4_eq, u_eq]));
        end % End of Linearization


        function motionPlot(obj,x,theta)
             w     = obj.W;                   %cart with;
             h     = obj.H;                   % Cart Height
             wh    = obj.Wh;                  %wheel position
             l     = obj.L;                   %Pendulum Length
             w1x = x -0.9*w/2+wh/2;           % Left Wheel  x Position
             w2x = x+0.9*w/2-wh/2;            % Right Wheel x position
             y=h/2 +wh/2;                     % Cart Center y Position
             py = y-l*cos(theta);             %Pendulum y Position
             px = x+l*sin(theta);             %Pendulum x Poaition
             cp = [x-(0.9*w)/2 wh/2 w h];     %Cart (Rectangle) Position
 
             line([-1 1],[0 0],'linewidth',3,'color','black');%Base Line
             hold on
             rectangle('Position',cp,'FaceColor','#D95319','EdgeColor',...
                'None','Curvature',0.1);                       % Cart
             rectangle('Position',[w1x 0 wh wh],'FaceColor','blue',...
                 'Curvature',1,'EdgeColor','None');            % Right Wheel
             rectangle('Position',[w2x 0 wh wh],'FaceColor','blue',...
             'Curvature',1,'EdgeColor','None');                % Left Wheel

 
 
             plot([x px],[y py],'LineWidth',3);                   % Pendulum 
             viscircles([x,y],0.02,'LineWidth',2,'color','white');% cart bob
             viscircles([px,py],0.01,'LineWidth',5,'Color','k');  % bob 
             axis equal
             axis([-1 1 -0.4 1]);
             grid minor
             hold off
        end
    end
 end
