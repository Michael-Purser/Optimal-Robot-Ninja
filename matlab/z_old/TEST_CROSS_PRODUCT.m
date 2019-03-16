% Test cross product:

clear;

a = [1;1;0];
N = 200;
phi = linspace(0,2*pi,N);

figure;
for k = 1:N
    b = [cos(phi(k));sin(phi(k));0];
    C = cross(a,b);
    plot(phi(k),C(3),'.'); hold on;
end
