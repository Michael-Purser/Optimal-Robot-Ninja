function plotGaussians2D(log,it,N)
% function that plots the gaussian landscape as 2D contour lines

% get data
vehicleState    = log.states{it};
localMeas       = log.localPlanners{it}.obstacleData;
Ghat            = log.localPlanners{it}.params.Ghat;
sigma           = log.localPlanners{it}.params.sigma;
W               = log.maps{it}.width;

H               = W/2; % exact half-width of gridmap

% transform obstacle data to global frame:
globalMeas = toWorldFrame(vehicleState(1:2),vehicleState(3),localMeas');

% reconstruct gaussian local map from MPC data:
[gmap,xVec,yVec] = getGMap(N,H,globalMeas',vehicleState(3),sigma,2);

figure;
hold all;
drawGaussians2D(gmap,xVec,yVec,Ghat,10);
axis equal;