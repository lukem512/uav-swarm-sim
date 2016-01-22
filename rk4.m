function [x, y, theta] = rk4(x, y, theta, v, mu, dt)
    % Runge-Kutta (rk4)
    [k1_x, k1_y, k1_theta] = f_continuous(theta, v, mu);
    [k2_x, k2_y, k2_theta] = f_continuous(theta + k1_theta*dt/2, v, mu);
    [k3_x, k3_y, k3_theta] = f_continuous(theta + k2_theta*dt/2, v, mu);
    [k4_x, k4_y, k4_theta] = f_continuous(theta + k3_theta*dt, v, mu);

    x = x+(k1_x+2*k2_x+2*k3_x+k4_x)*dt/6;
    y = y+(k1_y+2*k2_y+2*k3_y+k4_y)*dt/6;
    theta = theta+(k1_theta+2*k2_theta+2*k3_theta+k4_theta)*dt/6;

    theta = mod(theta, 360);
end

function [xdot, ydot, thetadot] = f_continuous(theta, v, mu)
    xdot = v*sind(theta);
    ydot = v*cosd(theta);
    thetadot = v*mu;
end