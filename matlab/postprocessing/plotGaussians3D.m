function plotGaussians3D(MPC,it,N)
% function that plots the 3D gaussian landscape

% get data
vehicleState    = MPC.log.states{it};
localMeas       = MPC.log.opts{it}.obst;
Ghat            = MPC.nav.opt.Ghat;
sigma           = MPC.nav.opt.sigma;
W               = MPC.nav.map.width;

H               = W/2; % exact half-width of gridmap

% transform obstacle data to global frame:
globalMeas = toWorldFrame(vehicleState(1:2),vehicleState(3),localMeas');

% reconstruct gaussian local map from MPC data:
[gmap,xVec,yVec] = getGMap(N,H,globalMeas',vehicleState(3),sigma,2);

figure;
hold all;
drawGaussians3D(gmap,xVec,yVec,Ghat);
axis equal;
