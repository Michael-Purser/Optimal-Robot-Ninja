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
eval(['load ./data/MPC',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(MPC.nav.veh),'.mat;']);
eval(['load ./data/env',num2str(MPC.nav.env),'.mat;']);

% initialization:
veh.dynamics.COG        = [0;0;2];
veh.sensor.noiseamp     = 0;
veh.sensor.freq         = 100;
veh.motors.fmax         = 3;
MPC.nav.globalStart     = [0;0;0];
MPC.nav.globalGoal      = [6;8;pi/2];
MPC.nav.currentState    = MPC.nav.globalStart;  % Robot starts at global start
MPC.nav.currentVelocity = [0;0];                % Robot starts from standstil
MPC.nav.tolerance       = 0.01;
MPC.nav.opt.solver      = 'sqp';
MPC.nav.kmax            = 1000;
MPC.nav.rebuild         = true;
MPC.log.logBool         = true;
MPC.log.exportBool      = false;


%% MPC LOOP

% Check that sit, veh, ... have been initialized correctly

% setup parametric optimization problem:
if MPC.nav.goalReached == false
    if MPC.nav.rebuild==true
        problemIpopt 	= optim_setup(MPC,veh,'ipopt');
        problemSqp      = optim_setup(MPC,veh,'sqp');
    else
        load('problemIpopt.mat');
        load('problemSqp.mat');
    end
    MPC.nav.problemIpopt = problemIpopt;
    MPC.nav.problemSqp   = problemSqp;
end

% select most restrictive dynamic constraints:
MPC = getDynamicLimits(MPC,veh);

while (MPC.nav.goalReached == false && MPC.nav.k<=MPC.nav.kmax)
    
    fprintf('\n');
    fprintf('MPC STEP %i \n',MPC.nav.k);
    
    % here the robot should localize
    % however we assume that location is perfectly known, so no
    % localization routine is implemented here
    
    % this part checks whether the robot is already within tolerance of the
    % final navigation goal
    % if this is the case, the loop breaks and the robot is considered
    % in the right final state, ie navigation is finished
    if norm([MPC.nav.currentState(1:2);1]-[MPC.nav.globalGoal(1:2);1])<MPC.nav.tolerance
        fprintf(2,'Vehicle within tolerance of final goal! \n');
        fprintf(2,'STOPPED \n');
        MPC.nav.goalReached = true;
        break;
    end
    
    % this part simulates the environment info gathering of the robot
    % you can either work from purely preloaded info, purely measured
    % real-time, or both
    fprintf('Measuring environment \n');
    env = relevantObst(MPC,veh,env);
    MPC = sensor(MPC,veh,env);
%     if MPC.nav.k>1
%         alpha = 8;
%         sit = processMeas(sit,alpha); % adapt sensor measurements:
%     end
    
    % check if computation of a global plan is necessary
    % if so, a new global plan is computed

    % now get the local goal on the global plan, and transform it to the
    % local coordinate frame to be used in the optimization problem
    MPC = getLocalGoal(MPC);

    % solve optimization problem
    fprintf('Calculating optimal trajectory \n');
    MPC = optim(MPC,veh);
    
    % update vehicle position using the actuator frequency and
    % back-simulation with added noise to simulate real-world effects
    fprintf('Advancing robot to next state \n');
    MPC = mpcNextState(MPC,veh,3,2);
    
    % update loop counter
    MPC.nav.k = MPC.nav.k + 1;
    
    % if required, log the MPC iteration
    if MPC.log.logBool == true
        MPC = logMPC(MPC);
    end
    
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
fprintf('Plotting solution \n');
mpc_plotSol(sit,veh,env,count,2,2);


