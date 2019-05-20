function plotLocalSol(sit,veh,it)

X       = sit.Sol.X{it};
meas    = sit.meas{it};
H       = veh.Sensor.horizon;
G_hat   = veh.Optim.G_hat;
N       = veh.Map.N;

% reconstruct gaussian local map from 'sit' data:
sit     = makeMap(sit,veh);
sit     = addMeasurementsToMap(sit,veh,2,it);
sit     = addGaussianToMap(sit,veh);
dx      = sit.Temp.dx;
map     = sit.Temp.map;
gmap    = sit.Temp.gmap;

% make plot limits for contour:
x_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);
y_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);

figure; 
hold all;
plot(X(1,:),X(2,:),'k.');
plot(-meas(:,2).*sin(meas(:,1)), meas(:,2).*cos(meas(:,1)),'.');
legend('Optimal trajectory','Measurements');
axis equal;
title('Measurements and optimal trajectory');

figure;
hold all;
contour(x_vec,y_vec,gmap',[G_hat G_hat],'LineWidth',2,'LineColor','r');
contour(x_vec,y_vec,gmap',10);
plot(X(1,:),X(2,:),'k.');
axis equal;
axis([-5 10 -5 10]);
title('Gaussians local contours and optimal trajectory');