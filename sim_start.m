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
nAgents = 8;
agents = cell(nAgents);

% Open new figure window
figure;
hold on

% Colour map
colours = [[0.859 0.251 0.251]; [0.969 0.408 0.408]; [1.0 0.588 0.588]; ...
    [0.737 0.122 0.122]; [0.585 0.035 0.035]; [1.0 0.0 0.0]; ...
    [0.988 0.102 0.102]; [1.0 0.266 0.266]; [0.784 0.0 0.0]];
drawAirspace = false;

% Plot history
nHistory = 3;

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
    controller.id = aa;
    controller.state = 1;
    controller.launched = false;
    controller.neighbour.id = 0;
    controller.neighbour.distance = inf;
    controller.cloud.inside = false;
    controller.cloud.lastp = false;
    controller.cloud.direction = '';

    %% Physical Robot state
    robot.x = 0;
    robot.y = 0;
    robot.theta = 0;
    
    % Historical positions, for drawing
    robot.history = zeros(nHistory,2);
    
    %% Store values
    agents{aa}.controller = controller;
    agents{aa}.robot = robot;
end

%% Initialise communications
channel = initChannel();

%% Main simulation loop
for kk=1:((30 * 60 * 60) / dt),
    
    % time
    t = t + dt;
    
    % All agents launched?
    if t > (30 * nAgents)
        launched = true;
    else
        launched = false;
    end
    
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
        
        % Set off one at a time
        if t > (aa-1) * 30
            
            % Decide where to move
            controller = decide(controller, p, msgs, launched, dt);

            % Update position estimates
            k = controller.v * controller.mu;
            controller.theta = controller.theta+(k+2*k+2*k+k)*dt/6;
            controller.theta = mod(controller.theta, 360);

            %% Physical Robot
            % Store history
            for jj = nHistory:-1:2
                robot.history(jj, :) = robot.history(jj-1, :);
            end
            robot.history(1, :) = [robot.x robot.y];

            % Move the robot
            robot = move(robot, controller.v, controller.mu, dt);
            
            %% Controller
            % Retrieve noisy location from GPS
            [controller.x,controller.y] = gps(robot);

            % Send location to other agents
            msg = [controller.x controller.y controller.theta ...
                controller.v controller.mu controller.cloud.inside];
            channel = simTransmit(msg, channel);
        end

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
        % Get values
        robot = agents{aa}.robot;
        
        % Make colour
        if agents{aa}.controller.cloud.inside
            colour = 'g';
        else
            colour = colours(mod(aa, size(colours, 1))+1, :);
        end
        
        % Plot robot location
        plot(robot.x,robot.y,'Color',colour,'Marker','o');

        % Plot orientation
        trail = [robot.x robot.y; robot.history];
        plot(trail(:,1), trail(:,2), 'Color',colour,'LineStyle',':');
        
        if drawAirspace
            viscircles([robot.x robot.y],50);
            viscircles([robot.x robot.y],100);
        end
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