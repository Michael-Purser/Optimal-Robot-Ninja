function makeMovFrame(sit,veh,env,nb,qf)

if qf<1
    error('quality factor qf must be > 1.0');
end

N_original = veh.Map.N;
veh.Map.N = ceil(qf*veh.Map.N);

p       = [sit.states{end}(1:2);1];
phi     = sit.states{end}(3);
X       = sit.Sol.X{end};
U       = sit.Sol.U{end};
T       = sit.Sol.T{end};
G       = sit.Sol.G{end};
x_final = [sit.goalState(1:2);1];
H       = veh.Sensor.horizon;
G_hat   = veh.Optim.G_hat;
n       = veh.Optim.n;
L       = veh.wheelBase;
N       = veh.Map.N;
nObst   = size(env.Obst.pos,2);

arc     = 0:0.01:2*pi;

% reconstruct gaussian local map from 'sit' data:
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

% plot vehicle and its path: transform from local to global coordinates
% before plotting:
Tr = homTrans(phi,p);
x = zeros(3,size(X,2));
for k = 1:size(X,2)
    x(:,k) = Tr*[X(1,k);X(2,k);1];
end
x(3,:) = X(3,:);

pr = x(:,nb);
or = x(3,nb);
plot(pr(1),pr(2),'ro','MarkerSize',3,'LineWidth',1.4);
plot(pr(1)+0.5*L*cos(arc), pr(2)+0.5*L*sin(arc),'r',...
            'LineWidth',1.5);
quiver(pr(1)+0.5*L*cos(or),pr(2)-0.5*L*sin(or),...
    U(1,nb)*sin(or),U(1,nb)*cos(or),'r-','LineWidth',1.4);
quiver(pr(1)-0.5*L*cos(or),pr(2)+0.5*L*sin(or),...
    U(2,nb)*sin(or),U(2,nb)*cos(or),'r-','LineWidth',1.4);
plot([pr(1)+0.5*L*cos(or), pr(1)-0.5*L*cos(or)],...
    [pr(2)-0.5*L*sin(or), pr(2)+0.5*L*sin(or)],'r-','LineWidth',1)
plot(x(1,:),x(2,:),'k.');
plot(x_final(1),x_final(2),'xr','LineWidth',1.4);

% axis constraints:
axis equal;
xl = min(x(1,:))-0.5*L-1;
xh = max(x(1,:))+0.5*L+1;
yl = min(x(2,:))-0.5*L-1;
yh = max(x(2,:))+0.5*L+1;
axis([xl xh yl yh]);

% title:
title('Optimal trajectory in global frame','FontSize',12);

% restablish original N:
veh.Map.N = N_original;