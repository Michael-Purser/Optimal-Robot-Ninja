function mpc_plotSolFail(sit,veh,env,it,varargin)

N_original = veh.Map.N;
veh.Map.N = 200;
states  = {sit.states{1:it}};
p       = [states{it}(1:2);1];
phi     = states{it}(3);
% X       = sit.Sol.X{it};
% U       = sit.Sol.U{it};
% T       = sit.Sol.T{it};
% G       = sit.Sol.G{it};
x_final = [sit.goalState(1:2);1];
H       = veh.Sensor.horizon;
G_hat   = veh.Optim.G_hat;
n       = veh.Optim.n;
N       = veh.Map.N;
nObst   = size(env.Obst.pos,2);

arc     = 0:0.01:2*pi;

% reconstruct gaussian local map from 'sit' data:
sit     = makeMap(sit,veh);
sit     = addMeasurementsToMap(sit,veh,2,it);
sit     = addGaussianToMap(sit,veh);
dx      = sit.Temp.dx;
gmap    = sit.Temp.gmap;

% x_loc_coords = linspace(-(H-dx/2),H-dx/2,2*N+1);
% y_loc_coords = linspace(-(H-dx/2),H-dx/2,2*N+1);
x_loc_coords = linspace(-(H-dx),H-dx,2*N+1);
y_loc_coords = linspace(-(H-dx),H-dx,2*N+1);
x_loc_coords = [x_loc_coords;ones(1,size(x_loc_coords,2));ones(1,size(x_loc_coords,2))];
y_loc_coords = [ones(1,size(y_loc_coords,2));y_loc_coords;ones(1,size(y_loc_coords,2))];

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

% plot gaussians from local map: transform to global coordinates before 
% plotting:
x_glob_coords = homTrans(0,p)*x_loc_coords;
y_glob_coords = homTrans(0,p)*y_loc_coords;
x_glob_coords = x_glob_coords(1,:);
y_glob_coords = y_glob_coords(2,:);
contour(x_glob_coords,y_glob_coords,gmap',[G_hat G_hat],'LineWidth',2,'LineColor','r');
contour(x_glob_coords,y_glob_coords,gmap',10);

% % plot path from global planner: transform from local to global coordinates
% % before plotting:
% Tr = homTrans(phi,p);
% if ext==1
%     x_g = zeros(3,size(path,1));
%     for k = 1:size(path,1)
%         x_g(:,k) = Tr*[path(k,1);path(k,2);1];
%     end
%     plot(x_g(1,:),x_g(2,:),'g.');
% end

% plot vehicle and its path: transform from local to global coordinates
% before plotting:
% x = zeros(3,size(X,2));
% for k = 1:size(X,2)
%     x(:,k) = Tr*[X(1,k);X(2,k);1];
% end

for k=1:size(states,2)
    plot(states{k}(1),states{k}(2),'ro-','MarkerSize',6,'LineWidth',1.4);
end
% plot(x(1,:),x(2,:),'k.');
plot(x_final(1),x_final(2),'xr','LineWidth',1.4);

% axis constraints:
axis equal;
axis([-2 12 -1 12]);

% title:
title('Vehicle path in global frame');


if nargin == 6
    
    % plot velocities and accelerations:
%     t_vec   = linspace(0,T,n+1);
%     figure;
%     subplot(2,2,1);
%     plot(t_vec,X(3,:)); xlabel('time [s]'); ylabel('v_x(t) [m/s]'); title('v_x(t)');
%     subplot(2,2,2);
%     plot(t_vec,X(4,:)); xlabel('time [s]'); ylabel('v_y(t) [m/s]'); title('v_y(t)');
%     subplot(2,2,3);
%     plot(t_vec(1:end-1),U(1,:)); xlabel('time [s]'); ylabel('a_x(t) [m/s^2]'); title('a_x(t)');
%     subplot(2,2,4);
%     plot(t_vec(1:end-1),U(2,:)); xlabel('time [s]'); ylabel('a_y(t) [m/s^2]'); title('a_y(t)');
    
%     % plot n_new and max(G) in all iterations:
%     figure;
%     subplot(1,2,1); hold all;
%     for k = 1:size(sit.nNew,2)
%         plot(k,sit.nNew{k},'b.-'); 
%         xlabel('iteration [-]'); ylabel('n_{new} [-]');
%         title('n_{new} over successive MPC iterations');
%     end
%     subplot(1,2,2); hold all;
%     for k = 1:size(sit.Sol.G,2)
%         plot(k,max(sit.Sol.G{k}),'b.-'); 
%         xlabel('iteration [-]'); ylabel('max(G) [-]');
%         title('max(G) over successive MPC iterations');
%     end
%     
%     % plot euclidian distance between successive MPC states:
%     figure;
%     hold all;
%     for k = 1:size(sit.states,2)-1
%         s1 = sit.states{k}(1:2);
%         s2 = sit.states{k+1}(1:2);
%         d = norm(s1-s2);
%         plot(k,d,'b.-'); 
%         xlabel('iteration [-]'); ylabel('d [m]');
%         title('Euclidian distance between successive MPC iterations');
%     end

end

% restablish original N:
veh.Map.N = N_original;

end
