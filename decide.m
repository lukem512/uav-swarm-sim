function controller = decide(controller, p)    
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
    
    % The UAV may fly no more than 1km from (0,0)
    % Pick 900 to be safe
    if pdist([controller.x, controller.y; 0, 0]) > 100
        % Fly back towards (0,0)
        controller.mu = -3;
    end
   
    % mu is deg/m
    controller.mu = -0.2;
    
    % Add to incloud array
    % TODO
    controller.incloud = [controller.incloud (p > 0.01)];
    
    % Are we inside the cloud?
    % If so, tighter circles
    if p > 0.01
        % tsmovavg(controller.incloud, 's', 
        controller.mu = -1;
    end
    
    % Are we at the boundary?
    if p > 0.8 && p < 1.2
        % Stop turning!
        controller.mu = 0;
        
        % Reduce speed, we're there!
        controller.v = 10;
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