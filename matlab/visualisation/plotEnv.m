function plotEnv(env,MPC)
% Function that plots the environment with robot and obstacles, in world
% reference frame.
% Known obstacles are plotted in blue, unknown obstacles in cyan.

arc     = 0:0.01:2*pi;

figure;
hold all;

% plot obstacles:
for i=1:size(env.obst,2)
    type = env.obst{i}.type;
    mode = env.obst{i}.mode;
    if strcmp(mode,'known')==1
        plotColor = 'b';
    elseif strcmp(mode,'unknown')==1
        plotColor = 'c';
    end
    
    % circular obstacle:
    if strcmp(type,'circle')==1
        pos = env.obst{i}.center;
        R = env.obst{i}.radius;
        plot(pos(1)+R*cos(arc), pos(2)+R*sin(arc), plotColor,...
            'LineWidth',1.5);
        
    % rectangular obstacle:
    elseif strcmp(type,'rectangle')==1
        pos = env.obst{i}.center;
        W = env.obst{i}.width;
        H = env.obst{i}.height;
        ori = env.obst{i}.orientation;
        C = rectangleCorners(pos,W/2,H/2,ori);
        C(:,end+1) = C(:,1);
        plot(C(1,:),C(2,:),plotColor,'LineWidth',1.5);
    end
end

% plot MPC info on fig (optional)
if nargin == 2
    % plot robot with its orientation:
    p   = MPC.nav.currentState;
    plot(p(1),p(2),'ro');
    quiver(p(1),p(2),-sin(p(3)),cos(p(3)),'r-');

    % plot desired final robot position:
    g   = MPC.nav.globalGoal;
    plot(g(1),g(2),'rx');
end

% axis constraints:
axis equal;

% title:
title('Environment in world frame');

end