function plotEnv(env,MPC)
% Function that plots the environment with robot and obstacles, in world
% reference frame.

nObst   = size(env.Obst.pos,2);
arc     = 0:0.01:2*pi;

figure;
hold all;

% plot obstacles:
for i=1:nObst
    type = env.Obst.type(i);
    data = env.Obst.data(:,i);
    pos  = env.Obst.pos(:,i);
    % circular obstacle:
    if type==1
        plot(pos(1)+data(1)*cos(arc), pos(2)+data(1)*sin(arc),'b',...
            'LineWidth',1.5);
    % rectangular obstacle:
    else
        C = rectangleCorners(pos,data(1),data(2),data(3));
        C(:,end+1) = C(:,1);
        plot(C(1,:),C(2,:),'b','LineWidth',1.5);
    end
end

if nargin == 2
    % plot robot with its orientation:
    p   = MPC.nav.currentState;
    plot(p(1),p(2),'ro');
    quiver(p(1),p(2),-sin(p(3)),cos(p(3)),'r-');

    % plot desired final robot position:
    g   = MPC.nav.globalGoal;
    plot(g(1),g(2),'rx');
end

% Set axis constraints:
axis equal;

% Title:
title('Environment in world frame');

end