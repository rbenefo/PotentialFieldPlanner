%% Setup
clear all
close all
addpath('utils')
addpath('maps')

%% Simulation Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%       Modify this part       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start = [0,0,0,0,0,0];
goal = [0,0,pi/2,0,0,0];

mapToUse = 'map7.txt';
dynSim = true;      % false for map only, set to true to enable dynamic obstacles

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%   Do not modify after this   %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load Map and Robot

load 'robot.mat' robot
map = loadmap(mapToUse);
map.start = start;
map.goal  = goal;

%% Setup the Simulation

% If lynx global doesn't exist in the base workspace start Lynx
if(~evalin('base','exist(''lynx'',''var'')'))        
    startLynx = 1;
else
    global lynx
    
    if ~ishandle(lynx.hLinks)
        startLynx = 1;
    else
        startLynx = 0;
    end
end

% Uncomment the correct line to run in either simulation or hardware
if startLynx
    lynxStart();
    % lynxStart('Hardware','Legend','Port','COM3');
    pause(1);
end

H = plotmap(map);
% Move the lynx to the start position
lynxServo(start);
pause(1);

%% Run the Simulation

% Initially not done and at start location
isDone = 0;
qCurr = start;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [1] For dynamic simulation. 
if dynSim
    tic
    oldtimeElasped = 0;  % store the old time
    timeElasped = 0;
    T = 0;  % total time
    show = 0;  % flag to show obstacle
    count = 0;  % show how many obstacle
    obs = map.obstacles;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%       Modify this part       %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    T_appear = 1;  % time between obstacles appearing
    T_stay = 2; % duration per obstacle
    numAppear = 100; % number of obstacles appearing
    dist = 20; % set the distance between LIDAR and obstacle, unit: mm

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%   Do not modify after this   %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    dynVar.T = T;
    dynVar.count = count;
    dynVar.show = show;
    dynFix.T_appear = T_appear;
    dynFix.T_stay = T_stay;
    dynFix.numAppear = numAppear;
    dynFix.dist = dist;
    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% While not finished, take a step
    while ~isDone

       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % [2] For dynamic simulation.
       if dynSim
           dynVar.T = dynVar.T + timeElasped;
           [dynVar,map,H] = dynamicObs(dynVar,dynFix,map,qCurr,robot,obs,H);
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



       % Calculate the potential field step
       [qNext, isDone] = potentialFieldStep_11(qCurr, map, robot);

       %%%%%%% Uncomment only if running pcode 
%        fname = 'simParams.json';
%        fid = fopen(fname);
%        simJSON = jsondecode(fscanf(fid, '%c', inf));
%        fclose(fid);
%     
%        [qNext, isDone] = potentialFieldStep_sol(qCurr, map, robot,simJSON);
    %    
    %    
    %    %%%%%%%

       % Take the step
       lynxServo(qNext);
       qCurr = qNext;
       % Pause for simulation
       pause(0.02);  % You may change this to speed up the sim or slow it down


       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       % [3] For dynamic simulation
       % time elapsed
       if dynSim
           timeElasped = toc - oldtimeElasped;
           oldtimeElasped = toc;
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
