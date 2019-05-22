% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeSit()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

% add relevant folders to path:
addpath('~/Downloads/casadi/install/matlab/');
addpath('./data/');
addpath('./globalPlanner/');
addpath('./localPlanner/');
addpath('./robot/');
addpath('./MPC/');
addpath('./other/');
addpath('./environment/');
addpath('./postprocessing/');

% situation:
sitStr = '1_1_1';

% load situation, environment and vehicle:
eval(['load ./data/MPC',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(MPC.vehicle),'.mat;']);
eval(['load ./data/env',num2str(MPC.environment),'.mat;']);

% manual parameter overrides:
veh.sensor.noiseamp                 = 0;
veh.sensor.freq                     = 100;
veh.motors.fmax                     = 3;
% MPC.nav.obstacleData.localGridDx    = 0.05;
MPC.globalStart                     = [0;0;0];
MPC.globalGoal                      = [9;9;0];
% MPC.nav.goalTolerance               = 0.01;
% MPC.nav.opt.horizon                 = 100;
% MPC.nav.opt.Ghat                    = 0.15;
% MPC.nav.opt.sigma                   = 0.1;
% MPC.nav.opt.solver                  = 'ipopt';
% MPC.nav.opt.maxDistBeta             = 3;
% MPC.nav.opt.globalPlanR             = 1.5;
% MPC.nav.kmax                        = 1000;
% MPC.nav.rebuild                     = true;
MPC.nav.preload                     = true;
% MPC.log.logBool                     = true;
% MPC.log.exportBool                  = false;
% MPC.nav.withLocalGrid               = true;


%% MPC LOOP

% Initialize the MPC loop
[MPC,env,globalPlanner,localPlanner,log] = initialize(MPC,env);

% Actual loop:
while (MPC.goalReached == false && MPC.k<=MPC.kmax)
    
    fprintf('\n');
    fprintf('MPC STEP %i \n',MPC.k);
    
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
    
    % this part simulates the environment info gathering of the robot
    % you can either work from purely measured data in real-time, or a
    % combination of measured and preloaded data
    MPC = getMeasurements(MPC,veh,env);
    MPC = transformMeasurements(MPC);
    
    % get the local plan:
    [MPC,globalPlanner,localPlanner] = getLocalPlan(MPC,veh,globalPlanner,localPlanner,log);
    
    % check if recomputation of a global plan is necessary
    % if so, a new global plan is computed
    
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
makeMPCAVI(log,veh,env);
close all;

% plots:
k = MPC.k-1;

plotMPCState(log,veh,env,k);
plotDynamics(log,k);
plotMPCStats(log);



