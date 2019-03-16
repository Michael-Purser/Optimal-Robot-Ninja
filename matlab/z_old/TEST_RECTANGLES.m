% Script to test functions defined on rectangles:

% Make a rectangle in global coordinate system:
p       = [2 2 1]';
theta   = pi/5;
dx      = 1;
dy      = 1;

% Get corner points of rectangles:
C = getRectangleCorners(p,dx,dy,theta);

% Get minimum distance from point to rectangle:
pos   = [4 2 1]';
[d,P] = getDistanceToRectangle(pos,p,dx,dy,theta);

% Get distance to edge given a certain rangefinder beam:
phi = pi/3;
H = 2;
p1 = C(:,1) - [0;0;1];
p2 = pos- [0;0;1];
v1 = C(:,1)-C(:,4);
v2 = [-H*sin(phi); H*cos(phi); 0];
[I,dEdge] = getDistanceToRectangleEdge(pos,v2,C);


% Debug:
fprintf('Minimum distance to rectangle edge: %f \n',d);
fprintf('Measured distance to rectangle edge: %f \n',dEdge);

figure;
hold all;
plot(pos(1), pos(2), 'bo');

plot([pos(1) pos(1)+v2(1)], [pos(2) pos(2)+v2(2)], 'k');

Sbeam = v2*(dEdge/norm(v2));
plot([pos(1) pos(1)+Sbeam(1)], [pos(2) pos(2)+Sbeam(2)], 'r');

C(:,end+1) = C(:,1);
plot(C(1,:),C(2,:),'g');
plot(P(1), P(2), 'ro');
axis equal