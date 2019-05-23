function makeMPC()
% Function that makes the structures containing the information about the
% different examined MPC runs.
% These are stored in .mat files for later access.
% Naming convention: first number is vehicle, second is environment, third
% is scenario using that particular vehicle and environment.
% e.g. 'MPC2_5_8' is the 8th examined case using vehicle 2 and environment
% 5.

%% GENERAL CASE:
MPC.vehicle         = 1;
MPC.environment     = 1;
MPC.globalStart     = [];
MPC.globalGoal      = [];
MPC.currentState    = [];       % Current robot state
MPC.currentVelocity = [];       % Current velocity signal

MPC.obstacleData.meas.localPolar        = [];      % Measurements from simulated sensor (polar)
MPC.obstacleData.meas.localCartesian    = [];      % Measurements in cartesian coordinates
MPC.obstacleData.meas.globalCartesian   = [];      % Measurements in cartesian coordinates
MPC.obstacleData.preloaded              = [];      % Preloaded sampled environment data
MPC.obstacleData.preloadedLocal         = [];      % Preloaded sampled environment data in local frame

MPC.goalTolerance   = 1e-2;     % Tolerance on final goal
MPC.goalReached     = false;    % Indicate if final goal is reached
MPC.k               = 1;        % MPC loop counter
MPC.kmax            = 1000;     % MPC loop counter maximum
MPC.m               = 0;        % Number of states advanced in current MPC step
MPC.preload         = true;     % Indicate wether to sort the obstacles between mapped & measured

MPC.map.width       = 0;        % Map width
MPC.map.height      = 0;        % Map height
MPC.map.dx          = 0;        % Map discretization along x
MPC.map.dy          = 0;        % Map discretization along y
MPC.map.Nw          = 0;        % Map width discretization steps
MPC.map.Nh          = 0;        % Map height discretization steps
MPC.map.values      = [];       % Map matrix
MPC.map.sigma       = 0.1;      % sigma for map inflation
MPC.map.inflated    = [];       % Map inflated with G-values
MPC.map.center      = [];       % Map center in world frame

%% situation 1_1_1:
MPC.vehicle         = 1;
MPC.environment     = 1;
MPC.globalStart     = [0;0;0];
MPC.globalGoal	    = [0;9;0];

save data/MPC1_1_1.mat MPC

% eval(['load ./data/env',num2str(MPC.nav.environment),'.mat;']);
% 
% plotSit(MPC,env);
% savefig(gcf,'figs/sitFigs/MPC1_1_1.fig');
% close(gcf);


% %% situation 1_4_1:
% MPC.nav.vehicle         = 1;
% MPC.nav.environment     = 4;
% MPC.nav.globalStart     = [0;0;0];
% MPC.nav.globalGoal	    = [9;9;0];
% 
% save data/MPC1_4_1.mat MPC
% 
% eval(['load ./data/env',num2str(MPC.nav.environment),'.mat;']);
% 
% plotSit(MPC,env);
% savefig(gcf,'figs/sitFigs/MPC1_4_1.fig');
% close(gcf);
% 
% 
% %% situation 1_5_1:
% MPC.nav.vehicle         = 1;
% MPC.nav.environment   	= 5;
% MPC.nav.globalStart  	= [0;0;0];
% MPC.nav.globalGoal	    = [9;9;0];
% 
% save data/MPC1_5_1.mat MPC
% 
% eval(['load ./data/env',num2str(MPC.nav.environment),'.mat;']);
% 
% plotSit(MPC,env);
% savefig(gcf,'figs/sitFigs/MPC1_5_1.fig');
% close(gcf);
% 
% 
% %% situation 1_8_1:
% MPC.nav.vehicle     	= 1;
% MPC.nav.environment  	= 8;
% MPC.nav.globalStart   	= [0;0;0];
% MPC.nav.globalGoal	    = [9;9;0];
% 
% save data/MPC1_8_1.mat MPC
% 
% eval(['load ./data/env',num2str(MPC.nav.environment),'.mat;']);
% 
% plotSit(MPC,env);
% savefig(gcf,'figs/sitFigs/MPC1_8_1.fig');
% close(gcf);
% 
% 
% %% situation 1_9_1:
% MPC.nav.vehicle     	= 1;
% MPC.nav.environment  	= 9;
% MPC.nav.globalStart   	= [-1;2;0];
% MPC.nav.globalGoal	    = [1;8;pi/4];
% 
% save data/MPC1_9_1.mat MPC
% 
% eval(['load ./data/env',num2str(MPC.nav.environment),'.mat;']);
% 
% plotSit(MPC,env);
% savefig(gcf,'figs/sitFigs/MPC1_9_1.fig');
% close(gcf);