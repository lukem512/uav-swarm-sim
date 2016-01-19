function controller = decide(controller, p, dt)
    % Make bounds object
    b = bounds();
    
    % Travel in a straight line at top speed
    controller.mu = 0;
    controller.v = b.maxv(dt);
    
    % Too close to the boundary?
    if pdist([controller.x,controller.y;0,0]) > 300
        if controller.steps > 0
            controller.steps = controller.steps - 1;
        else
			origin.x = 0;
			origin.y = 0;
            delta = atan2d(origin.x-controller.x,origin.y-controller.y) - controller.theta;
            [controller, steps] = turn(controller, delta, dt);
            controller.steps = steps - 1
        end
    end
	
	% TODO - more complex sensing behaviour
end

% Turn by delta degrees
function [controller, steps] = turn(controller, delta, dt)
    % Make bounds object
    b = bounds();
    
	% Normalise the angle
	delta = mod(delta, 360);
	if delta > 180
		delta = -180 + rem(delta, 180);
	elseif delta < -180
		delta = 180 - rem(delta * -1, 180);
	end
	
	% Nothing to do?
    if delta == 0
		steps = 1;
		return
    end
    
    % Delta is now in deg/s
    delta = delta / dt;
	
	% Something to do!
    steps = 0;
    found = false;
    while ~found
        steps = steps + 1;
        ang = delta / steps;
        for v = b.minv(dt):b.maxv(dt)
            mu = ang / v;
            if abs(mu) < b.maxmu(dt)
                found = true;
                break
            end
        end
    end
    controller.v = v;
    controller.mu = mu;
end