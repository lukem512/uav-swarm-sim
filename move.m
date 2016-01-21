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
    dt = 2;
    
    [k1_x, k1_y, k1_theta] = f_continuous(robot.theta, v, mu);
    [k2_x, k2_y, k2_theta] = f_continuous(robot.theta + k1_theta*dt/2, v, mu);
    [k3_x, k3_y, k3_theta] = f_continuous(robot.theta + k2_theta*dt/2, v, mu);
    [k4_x, k4_y, k4_theta] = f_continuous(robot.theta + k3_theta*dt, v, mu);
    
    robot.x = robot.x+(k1_x+2*k2_x+2*k3_x+k4_x)*dt/6;
    robot.y = robot.y+(k1_y+2*k2_y+2*k3_y+k4_y)*dt/6;
    robot.theta = robot.theta+(k1_theta+2*k2_theta+2*k3_theta+k4_theta)*dt/6;
    
    robot.theta = mod(robot.theta, 360);
end

function [xdot, ydot, thetadot] = f_continuous(theta, v, mu)
    xdot = v*sind(theta);
    ydot = v*cosd(theta);
    thetadot = v*mu;
end