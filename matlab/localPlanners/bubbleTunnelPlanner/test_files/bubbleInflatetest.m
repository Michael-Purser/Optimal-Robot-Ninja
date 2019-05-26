clear;
clc;

x = [0;0];

bubbleRmax = 2;

o1 = [0.5 pi];
o2 = [0.6 pi/4];
o3 = [0.2 -pi/4];
Opolar  = [o1;o2;o3];

% get obstacles in cartesian form
Ocartesian = zeros(size(Opolar));
for k = 1:size(Opolar,1)
   Ocartesian(k,1) = Opolar(k,1)*cos(Opolar(k,2));
   Ocartesian(k,2) = Opolar(k,1)*sin(Opolar(k,2));
end

% get first collision point
[~,i] = min(Opolar(:,1));
c1 = Ocartesian(i,:);

% tranform other data to new coordinate frame centered in c1 and with
% x-axis going through c1 and x
phi = atan2(x(2)-c1(2),x(1)-c1(1));

% get transformed measurements
Tr = homTrans(phi,c1);
Ocartesian2 = zeros(size(Ocartesian));
for k = 1:size(Ocartesian2,1)
    Temp = inv(Tr)*[Ocartesian(k,:)';1];
    Ocartesian2(k,:) = Temp(1:2)';
end

% remove row corresponding to c1 in new matrix
Ocartesian2(i,:) = [];

% remaining obstacles to polar in new coordinate system
Opolar2 = zeros(size(Ocartesian2));
for k = 1:size(Opolar2,1)
    Opolar2(k,1) = sqrt(Ocartesian2(k,1)^2 + Ocartesian2(k,2)^2);
    Opolar2(k,2) = atan2(Ocartesian2(k,2),Ocartesian2(k,1));
end

% for each obstacle, get circle radius
radii2 = zeros(size(Opolar2,1),1);
for k=1:size(radii2,1)
    radii2(k) = Opolar2(k,1)/(2*cos(Opolar2(k,2)));
end

% get smallest value:
[R,j] = min(radii2);

% get new center and plot the circle:
xNew2 = [R;0];
xNew = Tr*[xNew2;1];
xNew = xNew(1:2);


arc = 0:0.01:2*pi;
figure;
hold all;
plot(x(1),x(2),'x');
plot(Ocartesian(:,1),Ocartesian(:,2),'xr');
plot(xNew(1)+R*cos(arc),xNew(2)+R*sin(arc));
axis equal;