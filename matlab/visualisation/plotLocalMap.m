function plotLocalMap(sit,veh,it)
% Plot the vehicle local map, both without and with added gaussians.

H       = veh.Sensor.horizon;
G_hat   = veh.Optim.G_hat;
N       = veh.Map.N;

% make local map:
sit     = makeMap(sit,veh);
sit     = addMeasurementsToMap(sit,veh,2,it);
sit     = addGaussianToMap(sit,veh);
map     = sit.Temp.map;
gmap    = sit.Temp.gmap;
dx      = sit.Temp.dx;

% make plot limits for contour:
x_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);
y_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);

% make figure:
figure;

subplot(1,2,1);
hold all; 
for i = 1:size(map,1)
   for j = 1:size(map,2)
       if map(i,j)~=0
           plot(-H+i*dx,-H+j*dx, 'b.');
       end
   end
end
axis equal; 
axis([-H H -H H]);
xlabel('x_{v} [m]'); ylabel('y_{v} [m]');
title('Vehicle local discretized map');

subplot(1,2,2);
hold all;
contour(x_vec,y_vec,gmap',[G_hat G_hat],'LineWidth',2,'LineColor','r');
contour(x_vec,y_vec,gmap',10);
axis equal;
axis([-H H -H H]);
xlabel('x_{v} [m]'); ylabel('y_{v} [m]');
title('Contour plot with gaussians');

end