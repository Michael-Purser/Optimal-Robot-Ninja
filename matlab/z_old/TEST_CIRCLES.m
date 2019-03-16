% Script to test functions defined on circles:

close all;
clear;

% Make a circle in global coordinate system:
p       = [2 2 1]';
R       = 1;

% Get minimum distance from point to circle:
pos   = [0 2 1]';
[d,P] = getDistanceToCircle(pos,p,R);

% Get distance to edge given a certain rangefinder beam:
phi = -pi/2.7;
H   = 1.1;
v1  = p-pos;
v2  = [-H*sin(phi); H*cos(phi); 0];
[I,dEdge] = getDistanceToCircleEdge(v1,v2,R);





% Debug:
fprintf('Distance to circle: %f \n',d);
fprintf('Distance to circle edge: %f \n',dEdge);

figure;
hold all;

arc     = 0:0.01:2*pi;

plot(pos(1), pos(2), 'bo');
plot([pos(1) pos(1)+v2(1)], [pos(2) pos(2)+v2(2)], 'k');

Sbeam = v2*(dEdge/norm(v2));
% plot([pos(1) pos(1)+Sbeam(1)], [pos(2) pos(2)+Sbeam(2)], 'r');

plot(p(1)+R*cos(arc), p(2)+R*sin(arc),'g');
plot(P(1), P(2), 'ro');

axis equal