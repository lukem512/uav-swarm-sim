function [xest,yest] = gps(robot)
    % A GPS is accurate to between +/- 3m
    % on a good day
    % TODO: noise!
    xest = robot.x;
    yest = robot.y;
end