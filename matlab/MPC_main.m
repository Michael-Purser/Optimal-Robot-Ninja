% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeSit()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

% add relevant folders to path:
addpath('~/Downloads/casadi/install/matlab/');
addpath('./data/');
addpath('./rastar/');
addpath('./robot/');
addpath('./simulation/');
addpath('./visualisation/');

% situation:
sitStr = '1_1_1';

% load situation, environment and vehicle:
eval(['load ./data/MPC',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(MPC.nav.vehicle),'.mat;']);
eval(['load ./data/env',num2str(MPC.nav.environment),'.mat;']);

% parameter initialization:
veh.sensor.noiseamp                 = 0;
veh.sensor.freq                     = 100;
veh.motors.fmax                     = 3;
MPC.nav.obstacleData.localGridDx    = 0.05;
MPC.nav.globalStart                 = [0;0;0];
MPC.nav.globalGoal                  = [9;9;pi/2];
MPC.nav.currentState                = MPC.nav.globalStart;  % Robot starts at global start
MPC.nav.currentVelocity             = [0;0];                % Robot starts from standstill
MPC.nav.goalTolerance               = 0.001;
MPC.nav.opt.solver                  = 'ipopt';
MPC.nav.opt.maxDistBeta             = 3;
MPC.nav.opt.globalPlanR             = 2;
MPC.nav.kmax                        = 1000;
MPC.nav.rebuild                     = true;
MPC.nav.preload                     = true;
MPC.log.logBool                     = true;
MPC.log.exportBool                  = false;
MPC.nav.withLocalGrid               = true;
MPC.log.states{end+1}               = MPC.nav.currentState;
MPC.log.velocities{end+1}           = [0;0];


%% MPC LOOP

% Initialize the MPC loop
[MPC,env] = initialize(MPC,env,veh);

% Actual loop:
while (MPC.nav.goalReached == false && MPC.nav.k<=MPC.nav.kmax)
    
    fprintf('\n');
    fprintf('MPC STEP %i \n',MPC.nav.k);
    
    % (here the robot should localize
    % however we assume that location is perfectly known, so no
    % localization routine is implemented here)
    
    % this part checks whether the robot is already within tolerance of the
    % final navigation goal
    % if this is the case, the loop breaks and the robot is considered
    % in the right final state, ie navigation is finished
    if norm([MPC.nav.currentState(1:2);1]-[MPC.nav.globalGoal(1:2);1])<MPC.nav.goalTolerance
        fprintf(2,'Vehicle localized to be within tolerance of final goal! \n');
        fprintf(2,'STOPPED \n');
        MPC.nav.goalReached = true;
        break;
    end
    
    % this part simulates the environment info gathering of the robot
    % you can either work from purely measured data in real-time, or a
    % combination of measured and preloaded data
    fprintf('Measuring environment \n');
    MPC = sensor(MPC,veh,env);
    % optional: process the sensor meas (left out for now)
    % transform obstacle data to local frame and move to MPC data
    % structure:
    MPC = prepareObstacleData(MPC);
    
    % check if recomputation of a global plan is necessary
    % if so, a new global plan is computed

    % get the local goal on the global plan, and transform it to the
    % local coordinate frame to be used in the optimization problem
    MPC = getLocalStartAndGoal(MPC);

    % solve optimization problem
    fprintf('Calculating optimal trajectory \n');
    MPC = optim(MPC,veh);
    
    % update vehicle position using the actuator frequency and
    % back-simulation with added noise to simulate real-world effects
    fprintf('Advancing robot to next state \n');
    MPC = mpcNextState(MPC,veh,3,2);
    
    % if required, log the MPC iteration (this is useful for analysis and
    % visualizations later)
    if MPC.log.logBool == true
        MPC = logMPC(MPC);
    end
    
    % update loop counter
    MPC.nav.k = MPC.nav.k + 1;
end
 
% check that goal indeed reached:
if MPC.nav.goalReached == false || MPC.nav.k > MPC.nav.kmax
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
k = MPC.nav.k-1;
fprintf('Plotting solution \n');
mpc_plotSol(MPC,veh,env,k,false,1);


