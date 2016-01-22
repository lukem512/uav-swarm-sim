%
% Move the "physical" robot
% Not accessible from simulated controller.
%
% Luke Mitchell, Jan 2016
%
function robot = move(robot, v, mu, dt)
    % Make bounds object
    b = bounds();
    
    % Physically cap speed
    if v < b.minv(dt)
        v = b.minv(dt);
    elseif v > b.maxv(dt)
        v = b.maxv(dt);
    end
    
    % Physically cap turn
    if mu < b.minmu(dt)
        mu = b.minmu(dt);
    elseif mu > b.maxmu(dt)
        mu = b.maxmu(dt);
    end
    
    % Runge-Kutta (rk4)
    [robot.x, robot.y, robot.theta] = rk4(robot.x, robot.y, robot.theta, ...
        v, mu, dt);
end