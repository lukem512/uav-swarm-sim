%
% Simulated GPS module
%
% Luke Mitchell, Jan 2016
%
function [xest,yest] = gps(robot)
    % "A GPS is accurate to between +/- 3m
    % on a good day"
    maxnoise = 3;
    xest = robot.x + (rand() * maxnoise);
    yest = robot.y + (rand() * maxnoise);
end