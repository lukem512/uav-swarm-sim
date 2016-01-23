%
% Robot decision-making functions
% This contains a finite-state machine for most of the work.
%
% Luke Mitchell, Jan 2016
%
function controller = decide(controller, p, msgs, dt)
    % Make bounds object
    b = bounds();
    
    % Check for proximity
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
            controller.steps = 0;
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
            if norm([controller.x controller.y] - [0 0]) > 1000
                disp(fprintf('[Agent %d] I am TOO FAR from base!', controller.id));
            end
            
            % Where will we be next timestep?
            [x, y, theta] = rk4(controller.x, controller.y, ...
                controller.theta, controller.v, ...
                controller.mu, dt);
            next = [x y theta];
            
            % Will we be too far away?
            % Turn towards the origin
            if norm([next(1) next(2)] - [0 0]) > 900
                controller.target.x = 0;
                controller.target.y = 0;
                controller.steps = 0;
            end
            
            % Check too close to another agent?
            [controller, stop] = airspace(controller, next, msgs, dt);
            if stop
                return
            end
            
            % Are we in the cloud?
            if incloud(p)
                %disp(fprintf('[Agent %d] I am in the cloud!', controller.id));
                %controller.state = 3;
                %return
            end
            
            % Will we arrived?
            % Use a threshold to stop the UAV spinning around the targ.
            if pdist([next(1),next(2); ...
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
function [controller, stop] = airspace(controller, next, msgs, dt)
    stop = false;
   
    them = 0;
    index = 0;
    closest = inf;
    for jj = 1:size(msgs)
        % Get the other agent, skipping ourself
        if jj == controller.id
            continue
        end
        other = msgs{jj};
        
        % If it continued, where would it be?
        [x, y, theta] = rk4(other(1), other(2), other(3), ...
           other(4), other(5), dt);

        % Too close to our projected location?
        dist = round(norm([next(1) next(2)] - [x y]));
        if dist < closest
            closest = dist;
            index = jj;
            them = [x y theta];
        end

        % What to do?
        [controller, stop] = proximity(controller, next, them, index, closest);

        % Store the values
        controller.neighbour.id = index;
        controller.neighbour.distance = closest;
    end
end

% Too close?
function [controller, stop] = proximity(controller, next, them, index, closest)
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

        % Will our airspace intersect theirs?
        r = 100;
        [xout,yout] = circcirc(next(1),next(2),r,them(1),them(2),r);
        if ~isnan(xout)
            % Find the line bisecting the airspaces
            line = polyfit(xout, yout, 1);
            
            % Find two points on the line, a distance of 200 each
            l = 200;
            alpha = atand(line(1));
            dx = cos(alpha)*l;
            dy = sin(alpha)*l;
            
            p1 = [xout(1)+dx,yout(1)+dy];
            p2 = [xout(2)-dx,yout(2)-dy];
            
            % Visualise?
            plot(xout, yout, 'b--');
            plot(p1(1),p2(2),'bx');
            plot(p2(1),p2(2),'bx');
            
            % Compute delta for us for both points, and them for
            % both points.
            dus1 = norm([next(1),next(2)] - p1);
            dus2 = norm([next(1),next(2)] - p2);
            dthem1 = norm([them(1),them(2)] - p1);
            dthem2 = norm([them(1),them(2)] - p2);
            
            % If we're closer to one, go to it
            % If we're equidistant, lowest ID goes to first
            if dus1 < dthem1
                controller.target.x = p1(1);
                controller.target.y = p1(2);
            elseif dus2 < dthem2
                controller.target.x = p2(1);
                controller.target.y = p2(2);
            else
                if controller.id < index
                    controller.target.x = p1(1);
                    controller.target.y = p1(2);
                else
                    controller.target.x = p2(1);
                    controller.target.y = p2(2);
                end
            end
            controller.steps = 0;
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