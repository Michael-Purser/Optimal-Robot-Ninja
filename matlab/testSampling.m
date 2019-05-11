% make a circular obstacle
env.obst = {};
C1.name      = 'C1';
C1.type      = 'circle';
C1.mode      = 'known';
C1.center    = [-4;4;1];
C1.radius    = 3;
env.obst{end+1} = C1;

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