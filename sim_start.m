function sim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
% Luke Mitchell, Jan 2016
%

% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud2.mat'

% time and time step
t = 0;
dt = 2;

% open new figure window
figure
hold on % so each plot doesn't wipe the predecessor

%% Controller state
% Estimated position
controller.x = 0;
controller.y = 0;
controller.theta = 0;

% Commands
controller.v = 0;
controller.mu = 0;

% Memory
controller.steps = 0;
 
%% Physical Robot state
robot.x = 0;
robot.y = 0;
robot.theta = 0;

%% Main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;
    
    %% Take measurement
    % take measurement
    p = cloudsamp(cloud,robot.x,robot.y,t);
    
    %% Controller
    % Decide where to move
    controller = decide(controller, p, dt);
    
    % Update theta estimate
    controller.theta = controller.theta + controller.v*controller.mu;
    controller.theta = mod(controller.theta, 360);
    
    %% Physical Robot
    % Move the robot
    robot = move(robot, controller.v, controller.mu, dt);
    
    % Retrieve noisy location from GPS
    [controller.x,controller.y] = gps(robot);
    
    %% Visualise
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t,robot.x,robot.y,p))
        
    % plot robot location
    plot(robot.x,robot.y,'o');
 
    % plot orientaion
    plot([robot.x; robot.x+(sind(robot.theta)*50)],[robot.y; robot.y+(cosd(robot.theta)*50)],'-')
    
    % plot robot's estimated location
    plot(controller.x,controller.y,'ro')
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
    
end