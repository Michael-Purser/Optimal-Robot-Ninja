% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeSit()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

addpath('~/Downloads/casadi-matlabR2014a-v3.4.5/');
addpath('./data/');
addpath('./rastar/');
addpath('./robot/');
addpath('./simulation/');
addpath('./visualisation/');

% situation:
sitStr = '1_8_1';

% load situation, environment and vehicle:
eval(['load ./data/sit',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(sit.vehNum),'.mat;']);
eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

% manual overrides:
% veh.actuatorfMin    = 20;
% veh.Sensor.horizon  = 5;
veh.Sensor.noiseamp = 0;
% sit.startState      = [7;1;0;0];
% sit.states          = {sit.startState};
% sit.goalState       = [10;-10;0;0];


%% MPC LOOP

count               = 1;
max_it              = 1000;
goal_reached        = 0;
tol                 = 0.01;

% Initial goal check
if norm([sit.states{end}(1:2);1]-[sit.goalState(1:2);1])<tol
    fprintf('Goal already within tolerance! \n');
    fprintf('\n');
    fprintf('STOPPED \n');
    goal_reached = 1;
end

while (goal_reached == 0 && count<max_it)
    
    fprintf('\n');
    fprintf('MPC STEP %i \n',count);

    % sensor simulation
    fprintf('Simulating sensor... \n');
    env = relevantObst(sit,veh,env);
    sit = sensor(sit,veh,env);

    % make vehicle local map
    fprintf('Making local map... \n');
    sit = makeMap(sit,veh);
    sit = addMeasurementsToMap(sit,veh,2,count);

    % make G-landscape
    sit = addGaussianToMap(sit,veh);

    % express end goal in vehicle frame
    sit = getLocalGoal(sit);
    
    % get geometric path to goal
    fprintf('Global planner: \n');
    sit = globalPlanner(sit,veh);

    % solve optimization problem
    fprintf('Calculating optimal trajectory... \n');
    sit = optim(sit,veh);
    
    % update vehicle position
    sit = mpcNextState(sit,veh,2); % get and log nb of states advanced
    fprintf('Advancing robot %i states \n',sit.nNew{end});
    n_new = sit.nNew{end};
    newPos = homTrans(sit.states{end}(3),[sit.states{end}(1:2);1])*...
        [sit.Sol.X{end}(1:2,n_new);1];
    sit.states{end+1} = [newPos(1:2);sit.Sol.X{end}(3,n_new)];
    
    % update loop counter
    count = count + 1;
    
    % check if goal is reached
    if norm([sit.states{end}(1:2);1]-[sit.goalState(1:2);1])<tol
        fprintf('Goal reached! \n');
        goal_reached = 1;
    end
    
end



%% POST-PROCESSING:
% If failure, take second to last iteration
% count = count-1;
    
% check the obstacle constraint on solution
% sit.Sol.G = {};
% fprintf('Checking solution... \n');
% for k=1:count
%     sit = checkSolution(sit,veh,k);
% end

% make videos:
% makeVideos(sit,veh,env,sitStr,1);
% makeVideos(sit,veh,env,sitStr,2);
close all;

% plots:
% plotRangefinderData(sit,veh,count);
plotLocalMap(sit,veh,count);
plotGlobal(sit,veh,count);
% plotLocalSolution(sit,veh,count);
plotSolutionMPCFail(sit,veh,env,count,1,2);


