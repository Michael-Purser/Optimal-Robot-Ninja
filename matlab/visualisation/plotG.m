function plotG(sit,veh,it)
% Function that makes a 3D plot of the "gaussian landscape" around
% measurements, and adds a line representing the G_hat level.

N_original = veh.Map.N;
veh.Map.N = 100;
H         = veh.Sensor.horizon;
G_hat     = veh.Optim.G_hat;
N         = veh.Map.N;

% reconstruct gaussian local map from 'sit' data:
sit     = makeMap(sit,veh);
sit     = addMeasurementsToMap(sit,veh,2,it);
sit     = addGaussianToMap(sit,veh);
dx      = sit.Temp.dx;
gmap    = sit.Temp.gmap;

% make plot limits:
x_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);
y_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);

% plot:
figure;
hold all;
mesh(x_vec,y_vec,gmap'); axis off;
contour3(x_vec,y_vec,gmap',[veh.Optim.G_hat,veh.Optim.G_hat],'r','Linewidth',4);

% restablish original N:
veh.Map.N = N_original;

end