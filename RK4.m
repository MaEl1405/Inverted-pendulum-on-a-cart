function next_state = RK4(ODE_function, X0, t, dt)
    % ODE_function :    function handle that computes the derivative
    % X_current    :    the current state vector
    % t            :    current time (not used in the RK4 method)
    % dt           :    time step

    % Calculate the four intermediate steps
    k1 = dt * ODE_function(t, X0);
    k2 = dt * ODE_function(t + 0.5*dt, X0 + 0.5*k1);
    k3 = dt * ODE_function(t + 0.5*dt, X0 + 0.5*k2);
    k4 = dt * ODE_function(t + dt, X0 + k3);

    % Update the state using the weighted average of the four steps
    next_state= X0 + (1/6) * (k1 + 2*k2 + 2*k3 + k4);
end
