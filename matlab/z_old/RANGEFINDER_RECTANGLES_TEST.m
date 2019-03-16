% Script to test rangefinder execution on simple environment containing
% only rectangles.

close all;
clear;

% Make rectangles in global coordinate system: the data matrix has the same
% structure as in the main file.
P       = [2,3;2,3;1,1];
Data    = [1,1;1,1;0,pi/4];

% Specify robot position and orientation:
pos   = [1.5 3.5 1]';
ori   = -0.8*pi;

% Get distance to edge given a certain rangefinder beam: the angle phi is
% now defined in coordinate system LOCAL TO THE ROBOT:
dphi    = 0.01;
phi_max = pi/2;
N       = floor(2*phi_max/dphi);
H       = 3;
meas    = zeros(N,2);

for k = 1:N
    phi = -phi_max + k*dphi;
    meas(k,1) = phi;
    v2  = [-H*sin(phi+ori); H*cos(phi+ori); 0];
    % For each rectangle, get measured distance:
    dist = H*ones(1,size(Data,2));
    for i = 1:size(Data,2)
        p       = P(:,i);
        dx      = Data(1,i);
        dy      = Data(2,i);
        theta   = Data(3,i);
        C       = getRectangleCorners(p,dx,dy,theta);
        [I,d]   = getDistanceToRectangleEdge(pos,v2,C);
        if I==1
            dist(i) = d;
        end
    end
    % Smallest measured distance is measurement:
    if min(dist)==H
        meas(k,2) = 0;
    else
        meas(k,2) = min(dist);
    end
end

% Plot situation in global reference frame:
figure;
hold all;
for i = 1:size(Data,2)
    C = getRectangleCorners(P(:,i),Data(1,i),Data(2,i),Data(3,i));
    C(:,end+1) = C(:,1);
    plot(C(1,:),C(2,:),'g');
end
plot(pos(1), pos(2), 'bo');
plot([pos(1) pos(1)-sin(ori)], [pos(2) pos(2)+cos(ori)], 'b');
axis equal

% Plot measurements in robot reference frame:
figure;
hold all;
plot(0,0,'ro');
plot(-meas(:,2).*sin(meas(:,1)), meas(:,2).*cos(meas(:,1)),'.');
axis equal