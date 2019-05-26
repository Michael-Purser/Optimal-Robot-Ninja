% Main matlab script for the thesis
%
% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeMPC()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

% select situation:
sitStr = '1_9_1';

% select global end local planner:
globalPlannerStr  = 'relaxedAStar';
localPlannerStr   = 'bubbleTunnelPlanner';

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

% load situation, environment and vehicle:
eval(['load ./data/MPC',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(MPC.vehicle),'.mat;']);
eval(['load ./data/env',num2str(MPC.environment),'.mat;']);

% manual parameter overrides (add if you want):
veh.sensor.noiseamp                 = 0;
veh.sensor.freq                     = 100;
veh.motors.fmax                     = 10;
veh.motors.noiseamp                 = 0.0;
MPC.globalStart                     = [-5.3524;6.0316;-1.6110];
% MPC.globalGoal                      = [9;9;0];
MPC.kmax                            = 1000;
MPC.nav.preload                     = true;


%% MPC LOOP

% Initialize the MPC loop
[MPC,env,globalPlanner,localPlanner,log] = initialize(MPC,env);

% Actual loop:
while (MPC.goalReached == false && MPC.k<=MPC.kmax)
    
    fprintf('\n');
    fprintf('MPC STEP %i \n',MPC.k);
    
    % simulate the environment info gathering of the robot
    % you can either work from purely measured data in real-time, or a
    % combination of measured and preloaded data
    MPC = getMeasurements(MPC,veh,env);
    MPC = transformMeasurements(MPC);
    
    % (here the robot should localize
    % however we assume that location is perfectly known, so no
    % localization routine is implemented here)
    
    % this part checks whether the robot is already within tolerance of the
    % final navigation goal
    % if this is the case, the loop breaks and the robot is considered
    % in the right final state, ie navigation is finished
    MPC = checkGoalReached(MPC);
    if MPC.goalReached
        break
    end
    
    % get the local plan:
    [MPC,globalPlanner,localPlanner] = getLocalPlan(MPC,veh,globalPlanner,localPlanner,log);
    
    % check if recomputation of a global plan is necessary
    % if so, a new global plan is computed
    if localPlanner.sol.success == false
        error('Error while computing optimal solution');
    end
    
    % update vehicle position using the actuator frequency and
    % back-simulation with added noise to simulate real-world effects
    MPC = getStatesToAdvance(MPC,veh,localPlanner,3);
    MPC = sendActuationSignal(MPC,veh,localPlanner);
    
    % if required, log the MPC iteration (this is useful for analysis and
    % visualizations later)
    if log.logBool == true
        log = logMPC(MPC,localPlanner,globalPlanner,log);
    end
    
    % update loop counter
    MPC.k = MPC.k + 1;
end
 
% check that goal indeed reached:
if MPC.goalReached == false || MPC.k > MPC.kmax
    error('Maximum number of MPC iterations exceeded!');
end


%% POST-PROCESSING:

% make video:
if makeMov
    makeMPCAVI(log,veh,env);
    close all;
end

% plots:
k = MPC.k-1;

plotMPCState(log,veh,env,k);
plotDynamics(log,k);
plotMPCStats(log);



