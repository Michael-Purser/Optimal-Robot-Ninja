% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeSit()' in command line (in that
% order); they can be found in the 'simulation' folder.

% clear;
close all;
clc;

addpath('~/Downloads/casadi-matlabR2014a-v3.4.5/');
addpath('./data/');
addpath('./rastar/');
addpath('./robot/');
addpath('./simulation/');
addpath('./visualisation/');

% situation:
sitStr = '1_4_1';


%% SIMULATION

% load situation, environment and vehicle:
eval(['load ./data/sit',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(sit.vehNum),'.mat;']);
eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

% SPECIAL CASES: DO NOT ERASE

% % for testing global planner length bug:
% sit.startState = [7.7070;6.7039;0;0];
% % for testing add to map bug (may have to be run several times):
% sit.startState = [6.9786;3.6466;0;0];

% manual overrides:
sitMPC.orientation  = 0;
veh.Sensor.horizon  = 5;
veh.Sensor.noiseamp = 0;
sit.startState      = sitMPC.states{end};
sit.states          = {sit.startState};

% simulate sensor:
env = relevantObst(sit,veh,env);
sit = sensor(sit,veh,env);


%% VEHICLE

% make vehicle map:
sit = makeMap(sit,veh);
sit = addMeasurementsToMap(sit,veh,2,1);

% make G-landscape using sensor measurements:
sit = addGaussianToMap(sit,veh);

% express goal in vehicle frame:
sit = getLocalGoal(sit);

% get geometric path to goal:
sit = globalPlanner(sit,veh);

% get optimal trajectory:
sit = optim(sit,veh);


%% POST-PROCESSING 

% check obstacle constraints:
sit = checkSolution(sit,veh,1);

% plot environment (in global frame):
plotEnvironment(env,sit);

% plot sensor measurements (in vehicle frame):
plotRangefinderData(sit,veh,1);

% plot vehicle map:
plotLocalMap(sit,veh,1);

% plot global path used for initial guess:
plotGlobal(sit,veh,1);

% plot optimal trajectory (in both frames):
plotLocalSolution(sit,veh,1);
plotSolution(sit,veh,env);