% Script to test rangefinder execution on simple environment containing
% only circles.

close all;
clear;

% Make circles in global coordinate system:
P  = [2,3;2,3;1,1];
R  = [1,1];

% Specify robot position and orientation:
pos   = [1.5 3.5 1]';
ori   = -pi;

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
    % For each circle, get measured distance:
    dist = H*ones(size(R));
    for i = 1:length(R)
        p = P(:,i);
        v1 = p-pos;
        r = R(i);
        [I,d] = getDistanceToCircleEdge(v1,v2,r);
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
arc     = 0:0.01:2*pi;
for i = 1:length(R)
    p = P(:,i);
    r = R(i);
    plot(p(1)+r*cos(arc), p(2)+r*sin(arc),'g');
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