function controller = decide(controller, p, dt)
    % Make bounds object
    b = bounds();
    
    % Too close to the boundary?
    % Turn towards the origin
    if  controller.x > 900
    or  controller.y > 900
        controller.target.x = 0;
        controller.target.y = 0;
        disp('Returning to origin...');
        controller.state = 2;
    end
    
    % FSM
    switch controller.state
        % Pick a random target
        case 1
            a = -900;
            b = 900;
            r = (b-a).*rand(2,1) + a;
            controller.target.x = round(r(1));
            controller.target.y = round(r(2));
            fprintf('Flying to random location (%d,%d)\n', ...
                controller.target.x, controller.target.y);
            controller.state = 2;
            return
            
        % Fly to target
        case 2
            % Travel in a straight line at top speed
            controller.mu = 0;
            controller.v = b.maxv(dt);
            
            % Turns required?
            if controller.steps > 0
                controller.steps = controller.steps - 1;
            else
                [controller, steps] = face(controller, controller.target.x, controller.target.y, dt);
                controller.steps = steps - 1;
            end
            
            % Are we in the cloud?
            if incloud(p)
                disp('We are in the cloud!');
                controller.state = 3;
                return
            end
            
            % Have we arrived?
            % Use a threshold to stop the UAV spinning around the targ.
            if pdist([controller.x,controller.y; ...
                    controller.target.x,controller.target.y]) < 10
                disp('We have arrived!');
                controller.state = 1;
                return
            end
            
        % In the cloud
        case 3
            % Spin as quickly as possible
            controller.mu = b.maxmu(dt);
            controller.v = b.maxv(dt);
            return
        
        % Something went wrong
        otherwise
            disp('Something went wrong!');
    end
end

% Are we at the cloud boundary?
function in = incloud(p)
    in = p > 0.9 && p < 1.1;
end

% Turn towards coordinates
function [controller, steps] = face(controller, x, y, dt)
    delta = atan2d(x-controller.x,y-controller.y) - controller.theta;
    [controller, steps] = turn(controller, delta, dt);
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