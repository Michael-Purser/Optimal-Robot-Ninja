function [gmap,xVec,yVec] = getGMap(N,H,meas,phi,sigma,addingStrategy)
% Function that returns the map of G-values in world coordinates, for
% visualisation purposes

map     = zeros(2*N+1,2*N+1);
dx      = 2*H/(2*N+1);
map     = addMeasurementsToMap(map,dx,meas,phi,N,addingStrategy);
gmap    = addGaussianToMap(map,sigma,H,N);
xVec    = linspace(-(H-dx),H-dx,2*N+1);
yVec    = linspace(-(H-dx),H-dx,2*N+1);