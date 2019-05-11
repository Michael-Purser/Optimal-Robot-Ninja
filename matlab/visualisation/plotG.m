function plotG(MPC,veh,it)
% Function that makes a 3D plot of the "gaussian landscape" around
% measurements, and adds a line representing the G_hat level.

N         = 100;
H         = veh.sensor.horizon;
Ghat      = MPC.nav.opt.Ghat;

% reconstruct gaussian local map from 'sit' data:
map     = zeros(2*N+1,2*N+1);
dx      = 2*H/(2*N+1);
map     = addMeasurementsToMap(map,dx,MPC,N,2,it);
gmap    = addGaussianToMap(map,MPC,veh,N);

% make plot limits:
x_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);
y_vec = linspace(-(H-dx/2),H-dx/2,2*N+1);

% plot:
figure;
hold all;
mesh(x_vec,y_vec,gmap'); axis off;
contour3(x_vec,y_vec,gmap',[Ghat,Ghat],'r','Linewidth',4);

end