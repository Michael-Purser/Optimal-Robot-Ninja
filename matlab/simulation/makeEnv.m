function makeEnv()
% Function that makes different environments for the robot to drive around
% in. 
% The environments are made out of geometric shapes.
% They are stored in .dat files for later access.

env.obst.known.circles      = {};
env.obst.known.rectangles   = {};
env.obst.unknown.circles    = {};
env.obst.unknown.rectangles = {};


%% Environment 1: Two circles

env.obst.known.circles.C1.name      = 'circle1';
env.obst.known.circles.C1.center    = [-4;4;1];
env.obst.known.circles.C1.radius    = 3;

env.obst.known.circles.C2.name      = 'circle2';
env.obst.known.circles.C2.center    = [4;4;1];
env.obst.known.circles.C2.radius    = 3;

save data/env1.mat env


% plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env1.fig');
% close(gcf);


% %% Environment 2: Gap between two long narrow rectangles
% env.Obst.pos  = [-4, 4;
%                    3, 3;
%                    1, 1];
% env.Obst.type = [2, 2];
% env.Obst.data = [3, 3;
%                   0.8, 0.8;
%                   -pi/8, pi/8];
% 
% save data/env2.mat env
% 
% % plotEnvironment(env);
% savefig(gcf,'figs/envFigs/env2.fig');
% close(gcf);
% 
% 
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