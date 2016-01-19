function controller = decide(controller, p)
    % Too close to the boundary?
    if pdist([controller.x,controller.y;0,0]) > 300
        if controller.steps > 0
            controller.steps = controller.steps - 1;
        else
			origin.x = 0;
			origin.y = 0;
            delta = atan2d(origin.x-controller.x,origin.y-controller.y) - controller.theta;
            [controller, steps] = turn(controller, delta);
            controller.steps = steps - 1;
        end
    end
	
	% TODO - more complex sensing behaviour
    return

    % v is m/s
    controller.v = 20;
    
    % Saturate velocity
    % to between 10 and 20 m/s
    if controller.v < 10
        controller.v = 10;
    end
    if controller.v > 20
        controller.v = 20;
    end
    % mu is deg/m
    controller.mu = -0.2;

    % Are we inside the cloud?
    % If so, tighter circles
    if p > 0.01
        size(controller.incloud)
        tsmovavg(controller.incloud, 's', 5, 1)
        controller.mu = -1;
    end
    
    % Tighter circles if we're further inside
    if p > 0.3
        controller.mu = -1.5;
    end
    if p > 0.5
        controller.mu = -2;
    end
    if p > 0.8
        controller.mu = -2.5;
    end
    
    % Are we at the boundary?
    if p > 0.9 && p < 1.1
        % Stop turning!
        controller.mu = -6;
        
        % Reduce speed, we're there!
        controller.v = 20;
    end
    
    % Too deep!
    if p > 1.2
        controller.mu = -1;
    end
    
    % Saturate turn amount
    % to maximum 6 deg/m
    if controller.mu < -6
        controller.mu = -6;
    end
    if controller.mu > 6
        controller.mu = 6;
    end
end

% Turn by delta degrees
% This only works for dt = 1
function [controller, steps] = turn(controller, delta)
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
	
	% Something to do!
    steps = 0;
    found = false;
    while ~found
        steps = steps + 1;
        delta = delta / steps
        for v = 10:20
            mu = delta / v;
            if abs(mu) < 6
                found = true;
                break
            end
        end
    end
    controller.v = v;
    controller.mu = mu;
end