% make a circular obstacle
env.obst = {};
C1.name      = 'C1';
C1.type      = 'circle';
C1.mode      = 'known';
C1.center    = [4;4;1];
C1.radius    = 3;
env.obst{end+1} = C1;

R1.name         = 'R1';
R1.type         = 'rectangle';
R1.mode         = 'known';
R1.center       = [-2;3;1];
R1.width        = 3;
R1.height       = 0.8;
R1.orientation  = -pi/8;
env.obst{end+1} = R1;

% hardcode delta L
deltaL = 0.1;

% sample circular obstacle
center  = env.obst{1}.center;
R       = env.obst{1}.radius;

N       = ceil(2*pi*R/deltaL);
meas    = zeros(N,2);
for i=1:N
   meas(i,:) = [R*cos(i*2*pi/N) R*sin(i*2*pi/N)];
end

arc = 0:0.01:2*pi;
figure;
plot(R*cos(arc), R*sin(arc), 'b', 'LineWidth',1.5);
hold on;
plot(meas(:,1),meas(:,2),'ko');
axis equal;
title('Circle sampling in local coordinate axis');


% sample rectangularar obstacle
center  = env.obst{2}.center;
W       = env.obst{2}.width;
H       = env.obst{2}.height;

NW      = ceil(W/deltaL);
NH      = ceil(H/deltaL);
meas    = [];

% north side
meas = [meas;[linspace(-0.5*W,0.5*W,NW)' 0.5*H*ones(NW,1)]];
% south side
meas = [meas;[linspace(-0.5*W,0.5*W,NW)' -0.5*H*ones(NW,1)]];
% west side
meas = [meas;[-0.5*W*ones(NH,1) linspace(-0.5*H,0.5*H,NH)']];
% south side
meas = [meas;[0.5*W*ones(NH,1) linspace(-0.5*H,0.5*H,NH)']];

% remove duplicates
meas = unique(meas,'rows');

figure;
C = rectangleCorners([0;0],W/2,H/2,0.0);
C(:,end+1) = C(:,1);
plot(C(1,:),C(2,:),'b','LineWidth',1.5);
hold on;
plot(meas(:,1),meas(:,2),'ko');
axis equal;
title('Rectangle sampling in local coordinate axis');