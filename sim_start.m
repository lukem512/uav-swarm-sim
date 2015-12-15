function sim_start
%
% simulation example for use of cloud dispersion model
%
% Arthur Richards, Nov 2014
%

% load cloud data
% choose a scenario
% load 'cloud1.mat'
load 'cloud2.mat'

% time and time step
t = 0;
dt = 1;

% open new figure window
figure
hold on % so each plot doesn't wipe the predecessor

 %% Controller state
 controller = struct('x', 0, 'y', 0, 'theta', 90, 'v', 0, 'mu', 0, 'desiredtheta', 90, 'incloud', []);
 
 %% Physical Robot state
 robot = struct('x', 0, 'y', 0, 'theta', 90);

%% Main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;
    
    %% Take measurement
    % take measurement
    p = cloudsamp(cloud,robot.x,robot.y,t);
    
    %% Controller
    % Decide where to move
    controller = decide(controller, p);
    
    % Update theta estimate
    controller.theta = controller.theta + controller.v*controller.mu;
    
    %% Physical Robot
    % Move the robot
    robot = move(robot, controller.v, controller.mu);
    
    % Retrieve noisy location from GPS
    [controller.x,controller.y] = gps(robot);
    
    %% Visualise
    % clear the axes for fresh plotting
    cla
    
    % put information in the title
    title(sprintf('t=%.1f secs pos=(%.1f, %.1f)  Concentration=%.2f',t,robot.x,robot.y,p))
        
    % plot robot location
    plot(robot.x,robot.y,'o')
    
    % plot the cloud contours
    cloudplot(cloud,t)
    
    % pause ensures that the plots update
    pause(0.1)
    
end