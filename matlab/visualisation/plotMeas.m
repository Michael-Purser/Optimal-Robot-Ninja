function plotMeas(sit,veh,it)
% Function that plots sensor measurements in local vehicle frame

meas = sit.meas{it};
H    = veh.Sensor.horizon;
arc  = 0:0.1:2*pi;

figure;
hold all;

% plot measurements:
plot(-meas(:,2).*sin(meas(:,1)), meas(:,2).*cos(meas(:,1)),'.');

% plot vehicle, its orientation and its maximum sensor range:
plot(0,0,'ro');
quiver(0,0,0,1,'r-');
plot(H*cos(arc), H*sin(arc), 'g--');
title('Measurements in vehicle frame');
axis equal
axis([-H H -H H]);

end