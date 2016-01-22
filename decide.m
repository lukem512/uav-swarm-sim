%
% Robot decision-making functions
% This contains a finite-state machine for most of the work.
%
% Luke Mitchell, Jan 2016
%
function controller = decide(controller, p, msgs, dt, t)
    % Make bounds object
    b = bounds();
    
    % FSM
    switch controller.state
        % Pick a random target
        case 1
            a = -900;
            b = 900;
            r = (b-a).*rand(2,1) + a;
            controller.target.x = round(r(1));
            controller.target.y = round(r(2));
            controller.state = 2;
            
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
            
            % Too close to the boundary?
            % Turn towards the origin
            if norm([controller.x controller.y] - [0 0]) > 900
                controller.target.x = 0;
                controller.target.y = 0;
            end
            
            % Check too close to another agent?
            [controller, stop] = airspace(controller, msgs, dt, t);
            if stop
                return
            end
            
            % Are we in the cloud?
            if incloud(p)
                %disp(fprintf('[Agent %d] I am in the cloud!', controller.id));
                %controller.state = 3;
                %return
            end
            
            % Have we arrived?
            % Use a threshold to stop the UAV spinning around the targ.
            if pdist([controller.x,controller.y; ...
                    controller.target.x,controller.target.y]) < 20
                controller.state = 1;
                return
            end
            
        % In the cloud
        case 3
            % Spin as quickly as possible
            controller.mu = b.maxmu(dt);
            controller.v = b.maxv(dt);
            
            % Check whether we're still in the cloud
            % TODO
            return
            
        % Something went wrong
        otherwise
            disp(fprintf('[Agent %d] I am in an unknown state. Help!', controller.id));
    end
end

% Are we too close to another agent?
function [controller, stop] = airspace(controller, msgs, dt, t)
    stop = false;
    
    if t > 30
        them = 0;
        index = 0;
        closest = inf;
        for jj = 1:size(msgs)
            % Get the other agent, skipping ourself
            if jj == controller.id
                continue
            end
            other = msgs{jj};
            
            % Current position
            dist = round(norm([controller.x controller.y] - other(1:2)));
            if dist < 50
                disp(fprintf('[Agent %d] I am TOO CLOSE to agent %d!', controller.id, jj));
            end
            
            % If it continued, where would it be?
            [x, y, theta] = rk4(other(1), other(2), other(3), ...
               other(4), other(5), dt);
            dist = round(norm([controller.x controller.y] - [x y]));
            
            if dist < closest
                closest = dist;
                index = jj;
                them = [x y theta];
            end
        end

        % What to do?
        [controller, stop] = proximity(controller, index, closest, them, dt);

        % Store the values
        controller.neighbour.id = index;
        controller.neighbour.distance = closest;
    end
end

% Too close?
function [controller, stop] = proximity(controller, index, closest, them, dt)
    stop = false;
    
    % Is there a closest neighbour?
    if closest < inf
        % If it's the same neighbour as before,
        % and they're further away, leave things as they are
        if index == controller.neighbour.id
            if closest > controller.neighbour.distance
                return
            end
        end
        
        % Where will we be the next timestep?
        [x, y, theta] = rk4(controller.x, controller.y, controller.theta, ...
               controller.v, controller.mu, dt);
           
        % TODO - if our heading does not collide with them, do not turn

        % Will our airspace intersect theirs?
        r = 100;
        [xout,~] = circcirc(x,y,r,them(1),them(2),r);
        if ~isnan(xout)         
            delta = abs(them(3) - theta);
            if delta < 90
                % IF ANGLE IS GREATER, TURN RIGHT
                % ELSE, TURN LEFT
                if theta > them(3)
                    heading = -90;
                else
                    heading = 90;
                end
            elseif delta > 270
                % IF ANGLE IS GREATER, TURN LEFT
                % ELSE, TURN RIGHT
                if theta > them(3)
                    heading = 90;
                else
                    heading = -90;
                end
            else
                % TURN RIGHT
                heading = -90;
            end
            controller.target.x = x + sind(theta + heading)*400;
            controller.target.y = y + cosd(theta + heading)*400;
            stop = true;
        end
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