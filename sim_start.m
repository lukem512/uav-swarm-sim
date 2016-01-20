function sim_start
%
% Simulation of a swarm of UAVs tracking a cloud boundary.
% The boundary is taken as a density of 1PPM.
%
% Arthur Richards, Nov 2014
% Luke Mitchell, Jan 2016
%

%% Configuration
% Load cloud data
load 'cloud2.mat'

% Initial time and time step
t = 0;
dt = 2;

% Number of agents in swarm
nAgents = 10;
agents = cell(nAgents);

% Open new figure window
figure
hold on

%% Create the agents
for aa = 1:nAgents
    %% Controller state
    % Estimated position
    controller.x = 0;
    controller.y = 0;
    controller.theta = 0;

    % Commands
    controller.v = 0;
    controller.mu = 0;

    % Memory
    controller.state = 1;
    controller.steps = 0;

    %% Physical Robot state
    robot.x = 0;
    robot.y = 0;
    robot.theta = 0;
    
    %% Store values
    agents{aa}.controller = controller;
    agents{aa}.robot = robot;
end

%% Initialise communications
channel = initChannel();

%% Main simulation loop
for kk=1:1000,
    
    % time
    t = t + dt;
    
    for aa = 1:nAgents
        %% Get simulation values
        controller = agents{aa}.controller;
        robot = agents{aa}.robot;
        
        %% Take measurement
        % take measurement
        p = cloudsamp(cloud,robot.x,robot.y,t);

        %% Controller
        % Receive messages from other agents
        [msgs,channel] = simReceive(channel);
        
        % Decide where to move
        controller = decide(controller, p, msgs, dt);
        
        % Send location to other agents
        channel = simTransmit([controller.x controller.y],channel);

        % Update theta estimate
        controller.theta = controller.theta + controller.v*controller.mu;
        controller.theta = mod(controller.theta, 360);

        %% Physical Robot
        % Move the robot
        robot = move(robot, controller.v, controller.mu, dt);

        % Retrieve noisy location from GPS
        [controller.x,controller.y] = gps(robot);
        
         %% Store values
        agents{aa}.controller = controller;
        agents{aa}.robot = robot;
    end
    
    % Update messages
    channel = simChannel(channel);
    
    %% Visualise
    % Clear the axes for fresh plotting
    cla
    
    % Put information in the title
    title(sprintf('Simulation state at t=%.1f secs',t))
    
    for aa = 1:nAgents
        % Get value
        robot = agents{aa}.robot;
        
        % Plot robot location
        plot(robot.x,robot.y,'o');

        % Plot orientaion
        plot([robot.x; robot.x+(sind(robot.theta)*50)],[robot.y; robot.y+(cosd(robot.theta)*50)],'-')
    end
    
    % Plot the cloud contours
    cloudplot(cloud,t)
    
    % Pause ensures that the plots update
    pause(dt*0.1)
    
end

%% Communications
% All functions by Arthur Richards

function channel = initChannel()
    % initialize comms channel model
    channel.curMsgs = {};
    channel.newMsgs = {};

function [rxMsgs,channel] = simReceive(channel)
    % simulate receiving messages
    % simple broadcast model - just get everything
    rxMsgs = channel.curMsgs;
    
function channel = simTransmit(txMsgs,channel)
    % simulate transmitting a message
    % store it for next step
    channel.newMsgs = [channel.newMsgs; txMsgs];

function channel = simChannel(channel)
    % simple - everyone gets everything
    channel.curMsgs = channel.newMsgs;
    channel.newMsgs = {};