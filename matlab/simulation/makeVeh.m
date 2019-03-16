function makeVeh()
% Function that makes a struct containing parameters of the vehicle and the
% parameters/data the onboard computer of the vehicle has access to.
% This struct are stored in a .dat file for later access.

% All units are S.I. if not specified otherwise

% Vehicle data (geometry, dynamics, ...):
veh.wheelBase       = 0.2;
veh.actuatorfMin    = 3;

% Sensor data:
% TODO: Substitute real laser capabilities (Hokuyo??).
veh.Sensor.thetamax = pi;
veh.Sensor.horizon  = 5;
veh.Sensor.freq     = 30;
veh.Sensor.omega    = pi/4;
veh.Sensor.noiseamp = 0.01*veh.Sensor.horizon;

% Parameters for vehicle local map:
veh.Map.N           = 50;

% Parameters for optimization problem:
% TODO: Get velocity and acceleration limits of actual vehicle
veh.Optim.n         = 200;
veh.Optim.u_min     = -0.2;
veh.Optim.u_max     = 0.2;
veh.Optim.a_min     = -0.2;
veh.Optim.a_max     = 0.2;
veh.Optim.om_min    = -0.5;
veh.Optim.om_max    = 0.5;
veh.Optim.sigma_x   = 0.2;
veh.Optim.sigma_y   = 0.2;
veh.Optim.G_hat     = 2;
veh.Optim.T_init    = 5;

save data/veh1.mat veh

end