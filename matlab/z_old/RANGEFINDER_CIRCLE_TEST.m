% Script to test rangefinder execution on simple environment containing
% only 1 circle.

close all;
clear;

% Make a circle in global coordinate system:
p  = [2 2 1]';
R  = 1;

% Get minimum distance from point to circle:
pos   = [0 2 1]';
ori   = -pi/2;
[d,P] = getDistanceToCircle(pos,p,R);

% Get distance to edge given a certain rangefinder beam: the angle phi is
% now defined in coordinate system LOCAL TO THE ROBOT:
dphi    = 0.01;
phi_max = pi/2;
N       = floor(2*phi_max/dphi);
H       = 1.3;
v1      = p-pos;
meas    = zeros(N,2);

for k = 1:N
    phi = -phi_max + k*dphi;
    meas(k,1) = phi;
    v2  = [-H*sin(phi+ori); H*cos(phi+ori); 0];
    [~,dEdge] = getDistanceToCircleEdge(v1,v2,R);
    meas(k,2) = dEdge;
end

% Plot situation in global reference frame:
figure;
hold all;
arc     = 0:0.01:2*pi;
plot(pos(1), pos(2), 'bo');
plot([pos(1) pos(1)-sin(ori)], [pos(2) pos(2)+cos(ori)], 'b');
plot(p(1)+R*cos(arc), p(2)+R*sin(arc),'g');
plot(P(1), P(2), 'ro');
axis equal

% Plot measurements in robot reference frame:
figure;
hold all;
plot(-meas(:,2).*sin(meas(:,1)), meas(:,2).*cos(meas(:,1)),'.');
axis equal