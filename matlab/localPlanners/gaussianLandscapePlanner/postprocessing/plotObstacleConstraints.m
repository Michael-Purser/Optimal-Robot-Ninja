function plotObstacleConstraints(log,it)
% Function that calls the local planner obstacle plotting method.
% Acts as a 'bridge' between the general MPC plotting methods and the local
% planner displaying methods.
% In this case, it's based on the plotGaussians2D method.

% get data
vehicleState    = log.states{it};
localMeas       = log.localPlanners{it}.obstacleData;
Ghat            = log.localPlanners{it}.params.Ghat;
sigma           = log.localPlanners{it}.params.sigma;
W               = log.maps{it}.width;

H               = W/2; % exact half-width of gridmap
N               = 300;

% transform obstacle data to global frame:
globalMeas = toWorldFrame(vehicleState(1:2),vehicleState(3),localMeas');

% reconstruct gaussian local map from MPC data:
[gmap,xVec,yVec] = getGMap(N,H,globalMeas',vehicleState(3),sigma,2);

drawGaussians2D(gmap,xVec,yVec,Ghat,10);
