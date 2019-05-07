% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeSit()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

addpath('~/Downloads/casadi/install/matlab/');
addpath('./data/');
addpath('./rastar/');
addpath('./robot/');
addpath('./simulation/');
addpath('./visualisation/');

% situation:
sitStr = '1_1_1';

% load situation, environment and vehicle:
eval(['load ./data/sit',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(sit.vehNum),'.mat;']);
eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

% manual overrides:
veh.actuatorfMin    = 3;
% veh.Sensor.horizon  = 5;
veh.Sensor.noiseamp = 0;
veh.Optim.a_max     = 0.1;
veh.Optim.a_min     = -0.1;
% sit.startState      = [7;1;pi;0];
% sit.states          = {sit.startState};
sit.goalState       = [10;10;0];

% make global path to follow:
% pathManual = [[5;9],[7;3],sit.goalState(1:2)];
pathManual = [sit.goalState(1:2)];

sit.globalVisited = [];
sit.globalNotVisited = pathManual;


%% MPC LOOP

count               = 1;
max_it              = 1000;
goal_reached        = 0;
tol                 = 0.01;
tol_waypoints       = 0.5;

% Initial goal check
if norm([sit.states{end}(1:2);1]-[sit.goalState(1:2);1])<tol
    fprintf(2,'Vehicle already within tolerance of final goal! \n');
    fprintf(2,'STOPPED \n');
    goal_reached = 1;
end

while (goal_reached == 0 && count<=max_it)
    
    fprintf('\n');
    fprintf('MPC STEP %i \n',count);

    % sensor simulation
    fprintf('Simulating sensor \n');
    env = relevantObst(sit,veh,env);
    sit = sensor(sit,veh,env);

    % make vehicle local map
	% fprintf('Making local map \n');
	% sit = makeMap(sit,veh);
	% sit = addMeasurementsToMap(sit,veh,2,count);

    % make G-landscape
	% sit = addGaussianToMap(sit,veh);

    % express end goal in vehicle frame
    sit = getLocalGoal(sit);
    
    % get geometric path to goal
	% fprintf('Global planner: \n');
	% sit = globalPlanner(sit,veh);

    % solve optimization problem
    fprintf('Calculating optimal trajectory \n');
    sit = optim(sit,veh,count,'ipopt');
    
    % update vehicle position
    fprintf('Advancing robot to next state \n');
    sit = mpcNextState(sit,veh,3);
    
    % check if next waypoint has been reached
    if norm(sit.states{end}(1:2)-sit.globalNotVisited(:,1))<tol_waypoints
        fprintf('   Reached waypoint %i! \n',size(sit.globalVisited,2)+1);
        % update waypoints
        sit.globalVisited = [sit.globalVisited sit.globalNotVisited(:,1)];
        sit.globalNotVisited = sit.globalNotVisited(:,2:end);
    end
    
    % if in last iteration, update tolerance to real final tolerance:
    if size(sit.globalNotVisited,2)==1
        tol_waypoints = tol;
    end
    
    % check if goal is reached
    % TODO: for now it will stop too far from final goal!!
    if size(sit.globalNotVisited,2)==0
        fprintf('Goal reached! Yipeeeeee :-) \n');
        goal_reached = 1;
    else
        % update loop counter
        count = count + 1;
    end
    
end

% check that goal indeed reached:
if goal_reached == 0 || count > max_it
    error('Maximum number of MPC iterations exceeded!');
end


%% POST-PROCESSING:
    
% check the obstacle constraint on solution
sit.Sol.G = {};
fprintf('Checking solution \n');
for k=1:count
    sit = checkSolution(sit,veh,k);
end

% make videos:
% mpc_makeMov(sit,veh,env,sitStr,1);
% mpc_makeMov(sit,veh,env,sitStr,2);
close all;

% plots:
fprintf('Plotting solution \n');
mpc_plotSol(sit,veh,env,count,1,2);


