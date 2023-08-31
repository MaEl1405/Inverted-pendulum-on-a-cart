function [t,X] = odeSolver(f,tspan,h,X0,method)
    % Solve InvPendOnCart differential equations using the specified method.
    %
    % Parameters:
    % - f: Function handle representing the ODE system. It should accept
    %      the current time t and the current state X as arguments and 
    %      return the derivative of X at that time.
    % - tspan: A 2-element vector [t0, t_final] specifying the
    %          integration interval.
    % - h: Step size for explicit methods like Euler and Runge-Kutta.
    % - X0: Vector containing the initial values of the state variables.
    % - method: String specifying the ODE solving method ('Euler',
    %           'RungeKutta4', 'ode45').
    %
    % Returns:
    % - t: Vector of time points where the solution was computed.
    % - X: Matrix of solution values corresponding to each time point.

    % Calculate the number of time steps
    N = floor((tspan(end) - tspan(1))/h) + 1;

    % Generate a time vector
    t = tspan(1):h:tspan(end);

    % Initialize the solution matrix
    sN = numel(X0);
    X = zeros(sN,N);
    X(:,1)=X0;

    % Select the ODE solver method based on the input 'method'
    if strcmpi(method,'Euler')
        %Euler method
        for ii = 1:N-1
            X(:,ii+1) = X(:,ii) + h*f(t(ii),X(:,ii));
        end
        X = X';

    elseif strcmpi(method,'RungeKutta4')
        % 4th-order Runge-Kutta method
        for ii=1:N-1
            k1 = h*f(t(ii), X(:,ii));
            k2 = h*f(t(ii)+h/2, X(:,ii)+k1/2);
            k3 = h*f(t(ii)+h/2, X(:,ii)+k2/2);
            k4 = h*f(t(ii)+h, X(:,ii)+k3);

            X(:,ii+1) = X(:,ii) + (k1+2*k2+2*k3+k4)/6;
        end
        X=X';

    elseif strcmpi(method,'ode45')
        % MATLAB's ode45 method
        [t,X]=ode45(f,tspan,X0);
    else
        error(['Check your method. Supported methods are: euler,...' ...
            ' rungekutta4, ode45']);
    end
end