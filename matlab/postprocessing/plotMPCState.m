function plotMPCState(MPC,veh,env,it,N)

% get data
wheelBase       = veh.geometry.wheelBase;
vehicleState    = MPC.log.states{it};
vehicleVelocity = MPC.log.velocities{it};
localPlan       = MPC.log.opts{it}.sol.x;
localMeas       = MPC.log.opts{it}.obst;
states          = {MPC.log.states{1:it}};
localGoal       = MPC.log.opts{it}.goal;
globalStart     = MPC.nav.globalStart;
globalGoal      = MPC.nav.globalGoal;
globalPlan      = MPC.nav.globalPlan.worldCoordinates;
Ghat            = MPC.nav.opt.Ghat;
sigma           = MPC.nav.opt.sigma;
W               = MPC.nav.map.width;

H               = W/2; % exact half-width of gridmap
arc             = 0:0.01:2*pi;

% transform obstacle data to global frame:
globalMeas = toWorldFrame(vehicleState(1:2),vehicleState(3),localMeas');

% reconstruct gaussian local map from MPC data:
[gmap,xVec,yVec] = getGMap(N,H,globalMeas',vehicleState(3),sigma,2);

% transform solution to global frame
localPlanWorld = toWorldFrame(vehicleState(1:2),vehicleState(3),localPlan);

% transform local goal to world frame
localGoalWorld = toWorldFrame(vehicleState(1:2),vehicleState(3),localGoal);

% draw
figure;
hold all;

drawEnv(env.obst,arc);
drawGlobalPlan(globalPlan);
drawStartAndGoal(globalStart,globalGoal);
drawStates(states);
drawVeh(vehicleState,vehicleState(3),wheelBase,arc,vehicleVelocity);
drawGaussians2D(gmap,xVec,yVec,Ghat,10);
drawLocalPlan(localPlanWorld);
drawLocalGoal(localGoalWorld);

axis equal;
axis([-5 5 0 12]);