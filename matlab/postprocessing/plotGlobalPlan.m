function plotGlobalPlan(MPC,env)
% Function that plots the global plan in the environment

plan = MPC.nav.globalPlan.worldCoordinates;

arc = 0:0.1:2*pi;

figure;
hold all;
drawGlobalPlan(plan);
drawEnv(env.obst,arc);
axis equal;