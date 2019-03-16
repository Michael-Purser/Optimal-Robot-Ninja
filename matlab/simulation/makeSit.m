function makeSit()
% Function that makes the structures containing the information about the
% different examined situations.
% These are stored in .dat files for later access.
% Naming convention: first number is vehicle, second is environment, third
% is scenario using that particular vehicle and environment.
% e.g. 'sit2_5_8' is the 8th examined case using vehicle 2 and environment
% 5.

%% GENERAL CASE:
sit.vehNum          = 1;
sit.envNum          = 1;
sit.startState      = [];
sit.startControls   = [];
sit.goalState	    = [];
sit.states          = {sit.startState};
sit.controls        = {sit.startControls};
sit.localGoals      = {};
sit.globalVisited    = [];
sit.globalNotVisited = [];
sit.meas            = {};
sit.meas_tilde      = {[]};
sit.R               = {}; % logger for R-values to process measurements
sit.Init.T          = {5};
sit.Init.path       = {};
sit.Sol.X           = {};
sit.Sol.U           = {};
sit.Sol.T           = {};
sit.Sol.lamg        = {};
sit.Sol.G           = {}; % solution G-value logger
sit.nNew            = {}; % MPC states logger


%% situation 1_1_1:
sit.vehNum          = 1;
sit.envNum          = 1;
sit.startState      = [0;0;0];
sit.startControls   = [0;0];
sit.goalState	    = [0;10;0];
sit.states          = {sit.startState};
sit.controls        = {sit.startControls};
sit.Init.T          = {5};

save data/sit1_1_1.mat sit

eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

plotEnv(env,sit);
savefig(gcf,'figs/sitFigs/sit1_1_1.fig');
close(gcf);

%% situation 1_1_2:
sit.vehNum          = 1;
sit.envNum          = 1;
sit.startState      = [0;0;0];
sit.startControls   = [0;0];
sit.goalState	    = [10;10;0];
sit.states          = {sit.startState};
sit.controls        = {sit.startControls};
sit.Init.T          = {5};

save data/sit1_1_2.mat sit

eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

plotEnv(env,sit);
savefig(gcf,'figs/sitFigs/sit1_1_2.fig');
close(gcf);

%% situation 1_4_1:
sit.vehNum          = 1;
sit.envNum          = 4;
sit.startState      = [0;0;0];
sit.startControls   = [0;0];
sit.goalState	    = [10;10;0];
sit.states          = {sit.startState};
sit.controls        = {sit.startControls};
sit.Init.T          = {5};

save data/sit1_4_1.mat sit

eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

plotEnv(env,sit);
savefig(gcf,'figs/sitFigs/sit1_4_1.fig');
close(gcf);

%% situation 1_5_1:
sit.vehNum          = 1;
sit.envNum          = 5;
sit.startState      = [0;0;0];
sit.startControls   = [0;0];
sit.goalState	    = [10;10;0];
sit.states          = {sit.startState};
sit.controls        = {sit.startControls};
sit.Init.T          = {5};

save data/sit1_5_1.mat sit

eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

plotEnv(env,sit);
savefig(gcf,'figs/sitFigs/sit1_5_1.fig');
close(gcf);

%% situation 1_8_1:
sit.vehNum          = 1;
sit.envNum          = 8;
sit.startState      = [0;0;0];
sit.startControls   = [0;0];
sit.goalState	    = [10;10;0];
sit.states          = {sit.startState};
sit.controls        = {sit.startControls};
sit.Init.T          = {5};

save data/sit1_8_1.mat sit

eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

plotEnv(env,sit);
savefig(gcf,'figs/sitFigs/sit1_8_1.fig');
close(gcf);