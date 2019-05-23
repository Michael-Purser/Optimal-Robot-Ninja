% Main matlab script for the thesis
%
% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeMPC()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

% select situation:
sitStr = '1_1_1';

% select global end local planner:
globalPlannerStr  = 'relaxedAStar';
localPlannerStr   = 'gaussianLandscapePlanner';

% makeMovie:
makeMov = false;

% add relevant folders to path:
addpath('~/Downloads/casadi/install/matlab/');
addpath('./data/');
addpath(['./globalPlanners/',globalPlannerStr]);
addpath(['./localPlanners/',localPlannerStr]);
addpath(['./localPlanners/',localPlannerStr,'/postprocessing']);
addpath('./robot/');
addpath('./MPC/');
addpath('./other/');
addpath('./environment/');
addpath('./postprocessing/');

% call MPC loop:
[MPC,veh,env,globalPlanner,localPlanner,log] = MPC(sitStr,makeMov);