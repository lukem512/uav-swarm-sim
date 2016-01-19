function robot = move(robot, v, mu)
    % Physically cap speed
    if v < 10
        v = 10;
    elseif v > 20
        v = 20;
    end
    
    % Physically cap turn
    if mu < -6
        mu = -6;
    elseif mu > 6
        mu = 6;
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