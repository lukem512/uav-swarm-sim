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
    
    % Runge-Kutta
    %x_k1 = robot.x + v*sind(robot.theta);
    %y_k1 = robot.y + v*cosd(robot.theta);
    %theta_k1 = robot.theta + v*mu;
    
    %x = [x_k1; y_k1];
    %dt = 3.6;
    
    %k1 = x;
    %k2 = x+k1*dt/2;
    %k3 = x+k2*dt/2;
    %k4 = x+k3*dt;
    %xnew = x+(k1+2*k2+2*k3+k4)*dt/6
    
    % Apply values
    robot.x = robot.x + v*sind(robot.theta);
    robot.y = robot.y + v*cosd(robot.theta);
    robot.theta = mod(robot.theta + v*mu, 360);
    
    %robot.x = xnew(1);
    %robot.y = xnew(2);
end