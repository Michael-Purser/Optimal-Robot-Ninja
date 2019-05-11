function makeEnv()
% Function that makes different environments for the robot to drive around
% in. 
% The environments are made out of geometric shapes.
% They are stored in .dat files for later access.

env.obst      = {};
env.measured  = {};
env.mapped    = {};

%% Environment 1: Two circles

% reset env.obst cell array:
env.obst      = {};

C1.name      = 'C1';
C1.type      = 'circle';
C1.mode      = 'known';
C1.center    = [-4;4;1];
C1.radius    = 3;
env.obst{end+1} = C1;

C2.name      = 'C2';
C2.type      = 'circle';
C2.mode      = 'known';
C2.center    = [4;4;1];
C2.radius    = 3;
env.obst{end+1} = C2;

save data/env1.mat env


plotEnv(env);
savefig(gcf,'figs/envFigs/env1.fig');
close(gcf);


%% Environment 2: Gap between two long narrow rectangles

% reset env.obst cell array:
env.obst      = {};

R1.name         = 'R1';
R1.type         = 'rectangle';
R1.mode         = 'known';
R1.center       = [-2;3;1];
R1.width        = 3;
R1.height       = 0.8;
R1.orientation  = -pi/8;
env.obst{end+1} = R1;

R2.name         = 'R2';
R2.type         = 'rectangle';
R2.mode         = 'known';
R2.center       = [2;3;1];
R2.width        = 3;
R2.height       = 0.8;
R2.orientation  = pi/8;
env.obst{end+1} = R2;

save data/env2.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env2.fig');
close(gcf);


% %% Environment 3: two vertical rows of circles
% env.Obst.pos  = [-2, -2, -2, -2, -2, 2, 2, 2, 2, 2;
%                    0,  2,  4,  6,  8, 0, 2, 4, 6, 8;
%                    1,  1,  1,  1,  1, 1, 1, 1, 1, 1];
% env.Obst.type = [1,1,1,1,1,1,1,1,1,1];
% env.Obst.data = [1,1,1,1,1,1,1,1,1,1;
%                   0,0,0,0,0,0,0,0,0,0;
%                   0,0,0,0,0,0,0,0,0,0];
% 
% save data/env3.mat env
% 
% % plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env3.fig');
% close(gcf);
% 
% 
% %% Environment 4: arbitrarily placed circles and rectangles
% 
% % Obstacle data:
% env.Obst.pos  = [2,  5,  9,  9,  4.5;
%                   2,  7,  3,  6,  2;
%                   1,  1,  1,  1,  1];
% env.Obst.type = [1,  1,  1,  1,  2];
% env.Obst.data = [1,  2,  1,  1,  1;
%                   0,  0,  0,  0,  1;
%                   0,  0,  0,  0,  pi/12];
%               
% save data/env4.mat env
% 
% % plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env4.fig');
% close(gcf);
% 
% 
% %% Environment 5: corridor formed by by two long rectangles
% env.Obst.pos  = [-3, 3;
%                   5, 5;
%                   1, 1];
% env.Obst.type = [2, 2];
% env.Obst.data = [1, 1;
%                   5, 5;
%                   0, 0];
% 
% save data/env5.mat env
% 
% % plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env5.fig');
% close(gcf);
% 
% 
% %% Environment 6: Same as environment 5 but with a small circular obstacle at side of the corridor 
% env.Obst.pos  = [-3, 3, 1;
%                    5, 5, 6;
%                    1, 1, 1];
% env.Obst.type = [2, 2, 1];
% env.Obst.data = [1, 1, 0.7;
%                   5, 5, 0;
%                   0, 0, 0];
% 
% save data/env6.mat env
% 
% % plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env6.fig');
% close(gcf);
% 
% 
% %% Environment 7: Same as environment 5 but with a small circular obstacle in center of the corridor 
% env.Obst.pos  = [-3, 3, 0;
%                             5, 5, 6;
%                             1, 1, 1];
% env.Obst.type = [2, 2, 1];
% env.Obst.data = [1, 1, 0.7;
%                            5, 5, 0;
%                            0, 0, 0];
% 
% save data/env7.mat env
% 
% % plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env7.fig');
% close(gcf);
% 
% 
% %% Environment 8: Empty
% env.Obst.pos  = [];
% env.Obst.type = [];
% env.Obst.data = [];
% 
% save data/env8.mat env
% 
% % plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env8.fig');
% close(gcf);

end