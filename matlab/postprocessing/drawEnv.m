function drawEnv(obst,arc)
% Function that draws the environment obstacles, in world coordinate frame.
% A difference is made between known and unknown obstacles.

obstacleFillColor = [170 170 200]/255;

% plot obstacles:
for i=1:size(obst,2)
    type = obst{i}.type;
    mode = obst{i}.mode;
    if strcmp(mode,'known')==1
        plotStyle = 'k';
        obstacleFillColor = [170 170 200]/255;
    elseif strcmp(mode,'unknown')==1
        plotStyle = 'k--';
        obstacleFillColor = [210 210 210]/255;
    end
    
    % circular obstacle:
    if strcmp(type,'circle')==1
        pos = obst{i}.center;
        R   = obst{i}.radius;
        th  = 0:pi/50:2*pi;
        x_circle    = pos(1)+R*cos(th);
        y_circle    = pos(2)+R*sin(th);
        fill(x_circle, y_circle,obstacleFillColor);
        plot(pos(1)+R*cos(arc), pos(2)+R*sin(arc), plotStyle,...
            'LineWidth',1.5);
        
    % rectangular obstacle:
    elseif strcmp(type,'rectangle')==1
        pos = obst{i}.center;
        W   = obst{i}.width;
        H   = obst{i}.height;
        ori = obst{i}.orientation;
        C   = rectangleCorners(pos,W/2,H/2,ori);
        fill(C(1,:),C(2,:),obstacleFillColor);
        C(:,end+1) = C(:,1);
        plot(C(1,:),C(2,:),plotStyle,'LineWidth',1.5);
    end
end

% % plot MPC info on fig (optional)
% if nargin == 2
%     % plot robot with its orientation:
%     p   = MPC.nav.currentState;
%     plot(p(1),p(2),'ro');
%     quiver(p(1),p(2),-sin(p(3)),cos(p(3)),'r-');
% 
%     % plot desired final robot position:
%     g   = MPC.nav.globalGoal;
%     plot(g(1),g(2),'rx');
% end

% axis constraints:
% axis equal;

% title:
% title('Obstacles in world frame');

end