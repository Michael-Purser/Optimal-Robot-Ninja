function makeVeh()
% Function that makes a struct containing parameters of the vehicle and the
% parameters/data the onboard computer of the vehicle has access to.
% This struct are stored in a .mat file for later access.

% All units are S.I. if not specified otherwise

% Geometry and Dynamics
veh.geometry.wheelBase  = 0.2;
veh.geometry.height     = 0.3;

veh.dynamics.mass        = 15;
veh.dynamics.COG         = [0;0;0.2]; % in vehicle xyz
veh.dynamics.omLimits    = [-pi;pi];
veh.dynamics.velLimits   = [-0.2;0.2];
veh.dynamics.accLimits   = [-0.2;0.2];
veh.dynamics.jerkLimits  = [-0.5;0.5];

% Actuators
veh.motors.fmax       	= 3;
veh.motors.noiseamp    	= 0.03;
veh.motors.velLimits    = [-0.2,0.2];
veh.motors.accLimits    = [-0.2,0.2];
% Sensor:
veh.sensor.thetamax     = pi;
veh.sensor.horizon      = 5;
veh.sensor.freq         = 30;
veh.sensor.omega        = pi/4;
veh.sensor.noiseamp     = 0.01*veh.sensor.horizon;

% Parameters for vehicle local map:
veh.Map.N               = 50;

% Parameters for optimization problem:
% TODO: Get velocity and acceleration limits of actual vehicle
% veh.Optim.n         = 200;
% veh.Optim.u_min     = -0.2;
% veh.Optim.u_max     = 0.2;
% veh.Optim.a_min     = -0.2;
% veh.Optim.a_max     = 0.2;
% veh.Optim.om_min    = -0.5;
% veh.Optim.om_max    = 0.5;
% veh.Optim.sigma_x   = 0.2;
% veh.Optim.sigma_y   = 0.2;
% veh.Optim.G_hat     = 2;
% veh.Optim.T_init    = 5;

save data/veh1.mat veh

end