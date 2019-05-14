function makeSit()
% Function that makes the structures containing the information about the
% different examined MPC runs.
% These are stored in .mat files for later access.
% Naming convention: first number is vehicle, second is environment, third
% is scenario using that particular vehicle and environment.
% e.g. 'MPC2_5_8' is the 8th examined case using vehicle 2 and environment
% 5.

%% GENERAL CASE:
MPC.nav.veh             = 1;
MPC.nav.env             = 1;
MPC.nav.globalStart     = [];
MPC.nav.globalGoal      = [];
MPC.nav.currentState    = [];       % Current robot state
MPC.nav.currentVelocity = [];       % Current velocity signal

MPC.nav.globalPlan.gridCoordinates  = [];  % Global plan in gridmap coordinates
MPC.nav.globalPlan.worldCoordinates = [];  % Global plan in world coordinates

MPC.nav.map.width       = 0;        % Map width
MPC.nav.map.height      = 0;        % Map height
MPC.nav.map.dx          = 0;        % Map discretization along x
MPC.nav.map.dy          = 0;        % Map discretization along y
MPC.nav.map.Nw          = 0;        % Map width discretization steps
MPC.nav.map.Nh          = 0;        % Map height discretization steps
MPC.nav.map.values      = [];       % Map matrix
MPC.nav.map.inflated    = [];       % Map inflated with G-values
MPC.nav.map.center      = [];       % Map center in world frame

MPC.nav.obstacleData.meas.orig    = [];      % Measurements from simulated sensor (polar)
MPC.nav.obstacleData.meas.transLocal   = [];      % Measurements in cartesian coordinates
MPC.nav.obstacleData.meas.transGlobal   = [];      % Measurements in cartesian coordinates
MPC.nav.obstacleData.preloaded    = [];      % Preloaded sampled environment data

MPC.nav.tolerance       = 1e-2;     % Tolerance on final goal
MPC.nav.goalReached     = false;    % Indicate wether goal is reached
MPC.nav.k               = 1;        % Loop counter
MPC.nav.kmax            = 1000;     % Loop counter maximum
MPC.nav.m               = 0;        % Number of states advanced in this MPC step
MPC.nav.rebuild         = true;     % Indicate wether to rebuild (true) or load (false) the problem
MPC.nav.preload         = true;     % Indicate wether to sort the obstacles between mapped & measured
MPC.nav.problemIpopt    = 0;        % put here as compromise; limit memory when logging opt
MPC.nav.problemSqp      = 0;
MPC.nav.lastIndex       = 1;        % last index along the global plan (should be moved!!)

MPC.nav.opt.start         = [];
MPC.nav.opt.goal          = [];
MPC.nav.opt.horizon       = 200;
MPC.nav.opt.obst          = [];
MPC.nav.opt.dynLimits.vel = [];
MPC.nav.opt.dynLimits.acc = [];
MPC.nav.opt.dynLimits.om  = [];
MPC.nav.opt.sigma         = 0.1;
MPC.nav.opt.Ghat          = 2.0;
MPC.nav.opt.maxDist       = 0.1;
MPC.nav.opt.globalPlanR   = 2;

MPC.nav.opt.init.x      = [];
MPC.nav.opt.init.u      = [];
MPC.nav.opt.init.T      = 0;

MPC.nav.opt.sol.x       = [];
MPC.nav.opt.sol.u       = [];
MPC.nav.opt.sol.T       = 0;

MPC.nav.opt.solPrev.x   = [];
MPC.nav.opt.solPrev.u   = [];
MPC.nav.opt.solPrev.T   = 0;

MPC.nav.opt.solver      = 'sqp';

MPC.log.logBool         = false;    % if true, log previous solutions, states, ... (including CPU times).
MPC.log.exportBool      = false;    % if true, export casadi problem to a .casadi file
MPC.log.states          = {};
MPC.log.velocities      = {};
MPC.log.meas            = {};
MPC.log.globalPlans     = {};
MPC.log.opts            = {};
MPC.log.m               = {};
MPC.log.CPUtimes        = {};


%% situation 1_1_1:
MPC.nav.veh             = 1;
MPC.nav.env             = 1;
MPC.nav.globalStart     = [0;0;0];
MPC.nav.globalGoal	    = [0;10;0];
MPC.nav.currentState    = MPC.nav.globalStart;

save data/MPC1_1_1.mat MPC

eval(['load ./data/env',num2str(MPC.nav.env),'.mat;']);

plotEnv(env,MPC);
savefig(gcf,'figs/sitFigs/MPC1_1_1.fig');
close(gcf);

% %% situation 1_1_2:
% sit.vehNum          = 1;
% sit.envNum          = 1;
% sit.startState      = [0;0;0];
% sit.startControls   = [0;0];
% sit.goalState	    = [10;10;0];
% sit.states          = {sit.startState};
% sit.controls        = {sit.startControls};
% sit.Init.T          = {5};
% 
% save data/sit1_1_2.mat sit
% 
% eval(['load ./data/env',num2str(sit.envNum),'.mat;']);
% 
% plotEnv(env,sit);
% savefig(gcf,'figs/sitFigs/sit1_1_2.fig');
% close(gcf);
% 
% %% situation 1_4_1:
% sit.vehNum          = 1;
% sit.envNum          = 4;
% sit.startState      = [0;0;0];
% sit.startControls   = [0;0];
% sit.goalState	    = [10;10;0];
% sit.states          = {sit.startState};
% sit.controls        = {sit.startControls};
% sit.Init.T          = {5};
% 
% save data/sit1_4_1.mat sit
% 
% eval(['load ./data/env',num2str(sit.envNum),'.mat;']);
% 
% plotEnv(env,sit);
% savefig(gcf,'figs/sitFigs/sit1_4_1.fig');
% close(gcf);
% 
% %% situation 1_5_1:
% sit.vehNum          = 1;
% sit.envNum          = 5;
% sit.startState      = [0;0;0];
% sit.startControls   = [0;0];
% sit.goalState	    = [10;10;0];
% sit.states          = {sit.startState};
% sit.controls        = {sit.startControls};
% sit.Init.T          = {5};
% 
% save data/sit1_5_1.mat sit
% 
% eval(['load ./data/env',num2str(sit.envNum),'.mat;']);
% 
% plotEnv(env,sit);
% savefig(gcf,'figs/sitFigs/sit1_5_1.fig');
% close(gcf);
% 
% %% situation 1_8_1:
% sit.vehNum          = 1;
% sit.envNum          = 8;
% sit.startState      = [0;0;0];
% sit.startControls   = [0;0];
% sit.goalState	    = [10;10;0];
% sit.states          = {sit.startState};
% sit.controls        = {sit.startControls};
% sit.Init.T          = {5};
% 
% save data/sit1_8_1.mat sit
% 
% eval(['load ./data/env',num2str(sit.envNum),'.mat;']);
% 
% plotEnv(env,sit);
% savefig(gcf,'figs/sitFigs/sit1_8_1.fig');
% close(gcf);