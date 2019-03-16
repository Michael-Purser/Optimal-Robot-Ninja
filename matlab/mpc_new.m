% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeSit()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

addpath('~/Downloads/casadi/install/matlab/');
addpath('./data/');
addpath('./temp_190315/');
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
veh.Sensor.noiseamp = 0;
veh.Sensor.freq     = 100;
% veh.Optim.a_max     = 0.1;
% veh.Optim.a_min     = -0.1;
sit.startState      = [0;0;0];
sit.states          = {sit.startState};
% sit.goalState       = [8;0;pi/2];

% make global path to follow:
% pathManual = [[5;9],[7;3],sit.goalState(1:2)];

P = [-1 -sqrt(3)/2 -0.5 0 0.5 sqrt(3)/2 1; 0 0.5 sqrt(3)/2 1 sqrt(3)/2 0.5 0; 1 1 1 1 1 1 1];
P(1:2,:) = P(1:2,:)*5*sqrt(2);
for k=1:size(P,2)
    P(:,k) = homTrans(pi/4,[5 5])*P(:,k);
end

% pathManual = [P(1:2,:) sit.goalState(1:2)];
pathManual = [[7;0] sit.goalState(1:2)];

% pathManual = [[2;9] [6;3] sit.goalState(1:2)];

sit.globalVisited = [];
sit.globalNotVisited = pathManual;
sit.viewFactor      = 0.3; % percentage of sensor horizon used for waypoint switching;
sit.goalReached = 0;
sit.tol         = 0.01;


%% MPC LOOP

count               = 1;
max_it              = 1000;
tol_waypoints       = 0.5;

% Initial goal check
if norm([sit.states{end}(1:2);1]-[sit.goalState(1:2);1])<sit.tol
    fprintf(2,'Vehicle already within tolerance of final goal! \n');
    fprintf(2,'STOPPED \n');
    sit.goalReached = 1;
end

while (sit.goalReached == 0 && count<=max_it)
    
    fprintf('\n');
    fprintf('MPC STEP %i \n',count);

    % sensor simulation
    fprintf('Simulating sensor \n');
    env = relevantObst(sit,veh,env);
    sit = sensor(sit,veh,env);
    
    % adapt sensor measurements:
    if count>1
        alpha = 8;
        sit = processMeas(sit,alpha);
    end

    % express end goal in vehicle frame
    sit = getLocalGoal_new(sit,veh);

    % solve optimization problem
    fprintf('Calculating optimal trajectory \n');
    sit = optim_new(sit,veh,count,'ipopt');
    
    % update vehicle position
    fprintf('Advancing robot to next state \n');
    sit = mpcNextState(sit,veh,3);
    
    % check if goal is reached
    % TODO: for now it will stop too far from final goal!!
    if norm(sit.states{end}-sit.goalState)<=sit.tol
        sit.goalReached = 1;
        sit.globalVisited = [sit.globalVisited sit.goalState(1:2)];
        fprintf('Goal reached! Yipeeeeee :-) \n');
    else
        % update loop counter
        count = count + 1;
    end
    
end

% check that goal indeed reached:
if sit.goalReached == 0 || count > max_it
    error('Maximum number of MPC iterations exceeded!');
end


%% POST-PROCESSING:
    
% check the obstacle constraint on solution
% sit.Sol.G = {};
% fprintf('Checking solution \n');
% for k=1:count
%     sit = checkSolution(sit,veh,k);
% end

% make videos:
% mpc_makeMov(sit,veh,env,sitStr,count,1);
% mpc_makeMov(sit,veh,env,sitStr,2);
close all;

% plots:
fprintf('Plotting solution \n');
mpc_plotSol(sit,veh,env,count,2);


