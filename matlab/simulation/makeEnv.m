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

C1  = makeCircle('C1','known',[-4;4;1],3);
C2  = makeCircle('C2','known',[4;4;1],3);

env.obst{end+1} = C1;
env.obst{end+1} = C2;

save data/env1.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env1.fig');
close(gcf);


%% Environment 2: Gap between two long narrow rectangles

% reset env.obst cell array:
env.obst      = {};

R1 = makeRectangle('R1','known',[-2;3;1],3,0.8,-pi/8);
R2 = makeRectangle('R2','known',[2;3;1],3,0.8,pi/8);

env.obst{end+1} = R1;
env.obst{end+1} = R2;

save data/env2.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env2.fig');
close(gcf);


%% Environment 3: two vertical rows of circles

% reset env.obst cell array:
env.obst      = {};

C1  = makeCircle('C1','known',[-2;0;1],1);
C2  = makeCircle('C2','known',[-2;2;1],1);
C3  = makeCircle('C3','known',[-2;4;1],1);
C4  = makeCircle('C4','known',[-2;6;1],1);
C5  = makeCircle('C5','known',[-2;8;1],1);
C6  = makeCircle('C6','known',[2;0;1],1);
C7  = makeCircle('C7','known',[2;2;1],1);
C8  = makeCircle('C8','known',[2;4;1],1);
C9  = makeCircle('C9','known',[2;6;1],1);
C10 = makeCircle('C10','known',[2;8;1],1);

env.obst{end+1} = C1;
env.obst{end+1} = C2;
env.obst{end+1} = C3;
env.obst{end+1} = C4;
env.obst{end+1} = C5;
env.obst{end+1} = C6;
env.obst{end+1} = C7;
env.obst{end+1} = C8;
env.obst{end+1} = C9;
env.obst{end+1} = C10;

save data/env3.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env3.fig');
close(gcf);


%% Environment 4: arbitrarily placed circles and rectangles

% reset env.obst cell array:
env.obst      = {};

C1 = makeCircle('C1','known',[2;2;1],1);
C2 = makeCircle('C2','known',[5;7;1],2);
C3 = makeCircle('C3','known',[9;3;1],1);
C4 = makeCircle('C4','known',[9;6;1],1);
R1 = makeRectangle('R1','known',[4.5;2;1],1,1,pi/12);

env.obst{end+1} = C1;
env.obst{end+1} = C2;
env.obst{end+1} = C3;
env.obst{end+1} = C4;
env.obst{end+1} = R1;
              
save data/env4.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env4.fig');
close(gcf);


%% Environment 5: corridor formed by by two long rectangles

% reset env.obst cell array:
env.obst      = {};
              
R1 = makeRectangle('R1','known',[-3;5;1],1,5,0);
R2 = makeRectangle('R2','known',[3;5;1],1,5,0);

env.obst{end+1} = R1;
env.obst{end+1} = R2;

save data/env5.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env5.fig');
close(gcf);


%% Environment 6: Same as environment 5 but with a small circular obstacle at side of the corridor 

% reset env.obst cell array:
env.obst      = {};
              
R1 = makeRectangle('R1','known',[-3;5;1],1,5,0);
R2 = makeRectangle('R2','known',[3;5;1],1,5,0);
C1 = makeCircle('C1','known',[1;6;1],0.7);

env.obst{end+1} = R1;
env.obst{end+1} = R2;
env.obst{end+1} = C1;

save data/env6.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env6.fig');
close(gcf);


%% Environment 7: Same as environment 5 but with a small circular obstacle in center of the corridor 
                       
% reset env.obst cell array:
env.obst      = {};
              
R1 = makeRectangle('R1','known',[-3;5;1],1,5,0);
R2 = makeRectangle('R2','known',[3;5;1],1,5,0);
C1 = makeCircle('C1','known',[0;6;1],0.7);

env.obst{end+1} = R1;
env.obst{end+1} = R2;
env.obst{end+1} = C1;

save data/env7.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env7.fig');
close(gcf);


%% Environment 8: Empty

% reset env.obst cell array:
env.obst      = {};

save data/env8.mat env

plotEnv(env);
savefig(gcf,'figs/envFigs/env8.fig');
close(gcf);

end