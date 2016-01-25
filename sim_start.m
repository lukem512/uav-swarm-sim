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
nAgents = 6;
agents = cell(nAgents);

% How long to simulate for?
% 1 Minute
time = 1 * 60 * 60;
loops = time / dt;

% Open new figure window
figure;

% Colour map
colours = [[0.859 0.251 0.251]; [0.969 0.408 0.408]; [1.0 0.588 0.588]; ...
    [0.737 0.122 0.122]; [0.585 0.035 0.035]; [1.0 0.0 0.0]; ...
    [0.988 0.102 0.102]; [1.0 0.266 0.266]; [0.784 0.0 0.0]];
drawAirspace = false;

% Plot history
nHistory = 3;

% Plot cloud history
nCloudHistory = 50;

% Cloud history
cloudh = cell(loops,1);

% Concentration history
concentration = -1*ones(loops,1);

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
    controller.cloud.lastp = 0;
    controller.cloud.direction = '';
    controller.cloud.points = [];

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
for kk=1:loops,
    
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

    % Plot the agents
    subplot(4, 2, [1,3,5]);
    title(sprintf('Simulation State at t=%.1f',t));
    xlabel('x-coordinate');
    ylabel('y-coordinate');
    hold on
    cla
    
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
    cloudplot(cloud,t);
    
    % Plot where the agents think the cloud is
    subplot(4, 2, [2,4,6]);
    caxis([0, 50])
    c = colorbar('EastOutside');
    set(c,'YDir','reverse');
    ylabel(c,'Age of Estimate (dt)');
    xlabel('x-coordinate');
    ylabel('y-coordinate');
    axis equal;
    axis([min(cloud.x) max(cloud.x) min(cloud.y) max(cloud.y)]);
    title('Cloud Location Determined by UAVs');
    hold on
    cla
    
    cloudh(kk) = [];
    concentration(kk) = 0;
    for aa = 1:nAgents
        % Get values
        controller = agents{aa}.controller;
        
        % Get points within cloud
        if controller.cloud.inside
            cloudh{kk} = [cloudh{kk}; [controller.x controller.y]];
        end
        
        % Get the average concentration
        concentration(kk) = concentration(kk) + controller.cloud.lastp;
    end
    
    % Plot history
    %xs = [];
    %ys = [];
    %zs = [];
    
    if kk <= nCloudHistory
        nCloudHistoryToUse = kk - 1;
    else
        nCloudHistoryToUse = nCloudHistory;
    end  
    
    for jj = 0:nCloudHistoryToUse
        c = cloudh{kk-jj};
        if size(c,1) > 0
            s = [];
            col = [];
            for ii = 1:size(c,1)
                s = [s; nCloudHistoryToUse - jj]
                col = [col; jj]
            end
            if s == 0
               s = 1
            end
            c(:,1)
            scatter(c(:,1),c(:,2),s,col);
        end
        %if size(c,1) > 1
        %    xs = [xs; c(:,1).'];
        %    ys = [ys; c(:,2).'];
        %    z = [];
        %    for ii = 1:size(c,1)
        %        z = [z jj];
        %    end
        %    zs = [zs; z];
        %end
    end
    
    % Draw the contours!
    %if size(zs,1) > 1
        %contour(xs, ys, zs);
    %end
    
    % Divide by the number agents to ge the average
    concentration(kk) = concentration(kk) / nAgents;
    
    % Plot the average distance from the cloud
    subplot(4, 2, [7,8]);
    cla
    hold on
    plot(concentration);
    axis([1, loops, 0, 1]);
    title('Average Concentration at UAV Location');
    xlabel('Time (t)');
    ylabel('Average Concentration (ppm)');
    
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