function plotSol(sit,veh,env)

N_original = veh.Map.N;
veh.Map.N = 200;

p       = [sit.states{end}(1:2);1];
phi     = sit.states{end}(3);
path    = sit.Init.path{end};
X       = sit.Sol.X{end};
U       = sit.Sol.U{end};
T       = sit.Sol.T{end};
G       = sit.Sol.G{end};
x_final = [sit.goalState(1:2);1];
H       = veh.Sensor.horizon;
G_hat   = veh.Optim.G_hat;
n       = veh.Optim.n;
u_min   = veh.Optim.u_min;
u_max   = veh.Optim.u_max;
a_min   = veh.Optim.a_min;
a_max   = veh.Optim.a_max;
om_min  = veh.Optim.om_min;
om_max  = veh.Optim.om_max;
L       = veh.wheelBase;
N       = veh.Map.N;
nObst   = size(env.Obst.pos,2);

t_vec   = linspace(0,T(1),n+1);
arc     = 0:0.01:2*pi;

% construct discretized gaussian landscape from 'sit' data:
sit     = makeMap(sit,veh);
sit     = addMeasurementsToMap(sit,veh,2,1);
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
x_glob_coords = homTrans(phi,p)*x_loc_coords;
y_glob_coords = homTrans(phi,p)*y_loc_coords;
x_glob_coords = x_glob_coords(1,:);
y_glob_coords = y_glob_coords(2,:);
contour(x_glob_coords,y_glob_coords,gmap',[G_hat G_hat],'LineWidth',2,'LineColor','r');
contour(x_glob_coords,y_glob_coords,gmap',10);

% plot path from global planner: transform from local to global coordinates
% before plotting:
Tr = homTrans(phi,p);
x_g = zeros(3,size(path,1));
for k = 1:size(X,2)
    x_g(:,k) = Tr*[path(k,1);path(k,2);1];
end
plot(x_g(1,:),x_g(2,:),'g.');


% plot vehicle and its path: transform from local to global coordinates
% before plotting:
x = zeros(3,size(X,2));
for k = 1:size(X,2)
    x(:,k) = Tr*[X(1,k);X(2,k);1];
end

plot(p(1),p(2),'ro','MarkerSize',6,'LineWidth',1.4);
quiver(p(1),p(2),-0.7*sin(phi),0.7*cos(phi),'r-','LineWidth',1.4);
plot(x(1,:),x(2,:),'k.');
plot(x_final(1),x_final(2),'xr','LineWidth',1.4);

% axis constraints:
axis equal;
axis([0 11 0 11]);

% title:
title('Optimal trajectory in global frame','FontSize',12);



% plot velocities and accelerations:
figure;

subplot(2,2,1);
plot(t_vec(1:end-1),U(1,:)); hold on;
plot(t_vec(1:end-1),u_min*ones(1,size(t_vec,2)-1),'r--');
plot(t_vec(1:end-1),u_max*ones(1,size(t_vec,2)-1),'r--');
xlabel('time [s]'); ylabel('v_{right}(t) [m/s]'); title('v_{right}(t)');

subplot(2,2,2);
plot(t_vec(1:end-1),U(2,:)); hold on;
plot(t_vec(1:end-1),u_min*ones(1,size(t_vec,2)-1),'r--');
plot(t_vec(1:end-1),u_max*ones(1,size(t_vec,2)-1),'r--');
xlabel('time [s]'); ylabel('v_{left}(t) [m/s]'); title('v_{left}(t)');

subplot(2,2,3);
plot(t_vec(1:end-2),(n/T)*diff(U(1,:))); hold on;
plot(t_vec(1:end-2),a_min*ones(1,size(t_vec,2)-2),'r--');
plot(t_vec(1:end-2),a_max*ones(1,size(t_vec,2)-2),'r--');
xlabel('time [s]'); ylabel('a_{right}(t) [m^2/s]'); title('a_{right}(t)');

subplot(2,2,4);
plot(t_vec(1:end-2),(n/T)*diff(U(2,:))); hold on;
plot(t_vec(1:end-2),a_min*ones(1,size(t_vec,2)-2),'r--');
plot(t_vec(1:end-2),a_max*ones(1,size(t_vec,2)-2),'r--');
xlabel('time [s]'); ylabel('a_{left}(t) [m^2/s]'); title('a_{left}(t)');




% plot orientation and angular velocity:
figure;

subplot(1,2,1);
plot(t_vec,(180/pi)*X(3,:));
xlabel('time [s]'); ylabel('\phi(t) [°]'); title('\phi(t)');

subplot(1,2,2);
plot(t_vec(1:end-1),(180/pi)*(1/L)*(U(2,:)-U(1,:))); hold on;
plot(t_vec(1:end-1),(180/pi)*om_min*ones(1,size(t_vec,2)-1),'r--');
plot(t_vec(1:end-1),(180/pi)*om_max*ones(1,size(t_vec,2)-1),'r--');
xlabel('time [s]'); ylabel('\omega(t) [°/s]'); title('\omega(t)');



% plot G-values of path:
figure;
hold all;
plot(t_vec,G); xlabel('time [s]'); ylabel('G(x,y) [-]'); title('G-values of optimal trajectory');
plot(t_vec,G_hat*ones(1,length(t_vec)),'r');
legend('G-values of trajectory','Ĝ');
axis([0 t_vec(end) 0 max(1.2*G_hat,max(G))]);


% restablish original N:
veh.Map.N = N_original;