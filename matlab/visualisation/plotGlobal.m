function plotGlobal(sit,veh,it)

% map       = sit.Temp.gmap;
% dx        = sit.Temp.dx;
goal      = sit.localGoals{it}(1:2)';
path      = sit.Init.path{it};
H         = veh.Sensor.horizon;
G_hat     = veh.Optim.G_hat;

start = [0,0];

% make and retrieve local map:
sit     = makeMap(sit,veh);
sit     = addMeasurementsToMap(sit,veh,2,it);
sit     = addGaussianToMap(sit,veh);
map     = sit.Temp.gmap;
dx      = sit.Temp.dx;

figure;
hold all; 
for i = 1:size(map,1)
   for j = 1:size(map,2)
       if map(i,j)>=G_hat
           plot(-dx/2-H+i*dx,-dx/2-H+j*dx, 'k.');
       end
   end
end
plot(start(1),start(2),'ro');
plot(path(:,1),path(:,2),'g.');
plot(goal(1),goal(2), 'rx');
axis equal;
lim = max(max(abs(path(:))),H);
axis(1.1*[-lim lim -lim lim]); 
title('Global path for initial guess in vehicle frame');

end