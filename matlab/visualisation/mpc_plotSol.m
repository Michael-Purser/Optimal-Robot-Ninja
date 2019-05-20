function mpc_plotSol(MPC,veh,env,it,fail,varargin)

if fail
    %it      = MPC.nav.k - 1;
    %meas    = MPC.nav.obstacleData.meas.orig;
    meas    = MPC.nav.opt.obst;
else
    meas    = MPC.log.opts{it}.obst;
end

states      = {MPC.log.states{1:it}};
p           = [states{it}(1:2);1];
phi         = states{it}(3);
x_final     = [MPC.nav.globalGoal(1:2);1];
L           = veh.geometry.wheelBase;
H           = veh.sensor.horizon;
Ghat        = MPC.nav.opt.Ghat;
sigma       = MPC.nav.opt.sigma;
n           = MPC.nav.opt.horizon;
N           = 500;
arc         = 0:0.01:2*pi;
W           = MPC.nav.map.width;
H           = W/2;
globalPlan  = MPC.nav.globalPlan.worldCoordinates;
localGoal   = MPC.nav.opt.goal;
Rv          = MPC.nav.opt.globalPlanR;

% transform obstacle data to global frame:
T           = homTrans(phi,[p(1);p(2)]);
measGlobal  = zeros(size(meas));
for i=1:size(meas,1)
   A = T*[meas(i,:)';1];
   measGlobal(i,:) = A(1:2);
end

% reconstruct gaussian local map from MPC data:
map     = zeros(2*N+1,2*N+1);
dx      = 2*H/(2*N+1);
map     = addMeasurementsToMap(map,dx,measGlobal,phi,N,2);
gmap    = addGaussianToMap(map,sigma,H,N);

x_vec        = linspace(-(H-dx),H-dx,2*N+1);
y_vec        = linspace(-(H-dx),H-dx,2*N+1);

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

% plot gaussians from map:
contour(x_vec,y_vec,gmap',[Ghat Ghat],'LineWidth',2,'LineColor','r');
contour(x_vec,y_vec,gmap',10);

% plot global path:
plot(globalPlan(:,1),globalPlan(:,2),'g','LineWidth',1.4);

% plot vehicle global path view radius
plot(p(1)+Rv*cos(arc), p(2)+Rv*sin(arc), 'g--','LineWidth',1.5);

% plot vehicle and its path: transform from local to global coordinates
% before plotting:
Tr = homTrans(phi,p);
if fail==false
    X       = MPC.log.opts{it}.sol.x;
    U       = MPC.log.opts{it}.sol.u;
    T       = MPC.log.opts{it}.sol.T;
    x = zeros(3,size(X,2));
    for k = 1:size(X,2)
        x(:,k) = Tr*[X(1,k);X(2,k);1];
    end
    plot(x(1,:),x(2,:),'k.');
end

for k=1:size(states,2)
    plot(states{k}(1),states{k}(2),'r.','MarkerSize',6,'LineWidth',1.4);
end

% plot vehicle shape and arrows representing wheel velocities
plot(p(1)+0.5*L*cos(arc), p(2)+0.5*L*sin(arc),'k',...
'LineWidth',1.5);
plot([p(1)+0.5*L*cos(phi), p(1)-0.5*L*cos(phi)],...
    [p(2)-0.5*L*sin(phi), p(2)+0.5*L*sin(phi)],'k-','LineWidth',1)
quiver(p(1)+0.5*L*cos(phi),p(2)-0.5*L*sin(phi),...
    U(1,1)*sin(phi),U(1,1)*cos(phi),'k-','LineWidth',1.4);
quiver(p(1)-0.5*L*cos(phi),p(2)+0.5*L*sin(phi),...
    U(2,1)*sin(phi),U(2,1)*cos(phi),'k-','LineWidth',1.4);

% Plot local and global goal:
goal = Tr*[localGoal(1:2);1];
goal = [goal(1:2);localGoal(3)-phi];
plot(goal(1),goal(2),'ok','LineWidth',1.4);
quiver(goal(1),goal(2),sin(goal(3)),cos(goal(3)),'k','LineWidth',1.4);

plot(x_final(1),x_final(2),'xr','LineWidth',1.4);

% axis constraints:
axis equal;
axis([-2 12 -1 12]);

% title:
title('Vehicle path in global frame');


if nargin == 6 && fail == 0
    
    % plot velocities in wheels:
    t_vec   = linspace(0,T(1),n+1);
    figure;
    subplot(1,2,1);
    plot(t_vec(1:end-1),U(1,:)); xlabel('time [s]'); ylabel('v_{right}(t) [m/s]'); title('v_{right}(t)');
    subplot(1,2,2);
    plot(t_vec(1:end-1),U(2,:)); xlabel('time [s]'); ylabel('v_{left}(t) [m/s]'); title('v_{left}(t)');
    
    % plot max(G) in all iterations:
    figure;
    hold all;
    for k = 1:size(MPC.log.opts,2)
        plot(k,max(MPC.log.opts{k}.sol.Gvalues),'b.-'); 
        xlabel('iteration [-]'); ylabel('max(G) [-]');
        titlestr = {'max(G) over successive MPC iterations','(original measurements)'};
        title(titlestr);
    end
    
    % plot n_new and euclidian distance between successive MPC states:
    figure;
    subplot(1,2,1); hold all;
    for k = 1:size(MPC.log.opts,2)
        plot(k,MPC.log.m{k},'b.-'); 
        xlabel('iteration [-]'); ylabel('n_{new} [-]');
        titlestr = {'n_{new} over successive','MPC iterations'};
        title(titlestr);
    end
    subplot(1,2,2); hold all;
    for k = 1:size(MPC.log.states,2)-1
        s1 = MPC.log.states{k}(1:2);
        s2 = MPC.log.states{k+1}(1:2);
        d = norm(s1-s2);
        plot(k,d,'b.-'); 
        xlabel('iteration [-]'); ylabel('d [m]');
        titlestr = {'Euclidian distance between','successive MPC states'};
        title(titlestr);
    end
    
    % Plot 3D gaussian landscape
    figure;
    hold all;
    mesh(x_vec,y_vec,gmap'); axis off;
    contour3(x_vec,y_vec,gmap',[Ghat,Ghat],'r','Linewidth',4);

end


end
