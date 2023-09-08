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
%       - b: Viscous coefficient for the cart (N*m/rad/s)
%       - c: Viscous friction coefficient for cart displacement
%       
%
%   Methods:
%       - computeDynamics: Compute the system dynamics.
%
%   Example:
%       pendulum = InvPendOnCart(0.1,0.3,0.5,0.7,0.00008,0.0007);
%
classdef InvPendOnCart
    properties
        m       % bob mass
        M       % Cart mass
        L       % Pendulum length
        b       % Viscous Damping Coefficient (Pendulum)
        c       % Friction damping (Cart)
        I       % Mass moment of inertia
    end

    properties(Constant)
        g=9.81; % gravitational acceleration

        % properties for animation
        W =0.32      % cart with;
        H = 0.1;     % Cart Height
        Wh = 0.05;   % wheel position
    end

    methods
        % Constructor method
        function obj = InvPendOnCart(mPend,mCart,L,v_frict,v_damp,Inertia)
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
            obj.L = L;
            obj.b = v_damp;
            obj.c = v_frict;
            obj.I = Inertia;

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
            dx = zeros(4,1);
           
            x_dot = X(2);
            Sth = sin(X(3));
            Cth = cos(X(3));
            theta_dot = X(4);
         

            D = obj.m^2 * obj.L^2 * Sth^2  + obj.m * obj.M * obj.L^2 ...
                +(obj.M+ obj.m) * obj.I;


            dx(1,1) = X(2);
            dx(2,1) = (obj.b * obj.m * obj.L*theta_dot * Cth  + obj.m^2*obj.L^2*obj.g ...
                *Sth * Cth  + (obj.I + obj.m * obj.L^2)*...
                (u - obj.c * x_dot + obj.m * obj.L * theta_dot^2 * Sth) )/D;
            dx(3,1) = X(4);
            dx(4,1) = -(u * obj.m * obj.L * Cth - obj.c * obj.m * obj.L * ...
                x_dot * Cth + obj.m^2 * obj.L^2 ...
                *theta_dot^2 * Sth * Cth + (obj.m + obj.M ) * (obj.b ...
                * theta_dot + obj.m * obj.g * obj.L * Sth))/D;    

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
            Sth = sin(X(3));
            Cth = cos(X(3));
            x_dot = X(2);
            theta_dot = X(4);

            % Calculate the denominator factor D

            % Define the dynamics equations f(1) to f(4)
            D = obj.m^2 * obj.L^2 * Sth^2  + obj.m * obj.M * obj.L^2 ...
                +(obj.M+ obj.m) * obj.I;

            f(1) = X(2);
            f(2) = (obj.b * obj.m * obj.L * Cth  + obj.m^2*obj.L^2*obj.g ...
                *Sth * Cth  + (obj.I + obj.m * obj.L^2)*...
                (u - obj.c * x_dot + obj.m * obj.L * theta_dot^2 * Sth) )/D;
            f(3) = X(4);
            f(4) = -(u * obj.m * obj.L * Cth - obj.c * obj.m * obj.L * ...
                x_dot * Cth + obj.m^2 * obj.L^2 ...
                *theta_dot^2 * Sth * Cth + (obj.m + obj.M ) * (obj.b ...
                * theta_dot + obj.m * obj.g * obj.L * Sth))/D;

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
 
             line([-5 5],[0 0],'linewidth',3,'color','black');%Base Line
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
             
             x_min = x - 1.1 * w / 2;
             x_max = x + 1.1 * w / 2;
             y_min = -0.2;
             y_max = h + 0.2;

             axis([-1+x_min 1+x_max -0.5+y_min 0.5+y_max]);
             grid minor
             hold off
        end
    end
 end
