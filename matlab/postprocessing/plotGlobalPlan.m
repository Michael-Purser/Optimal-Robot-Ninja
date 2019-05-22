function plotGlobalPlan(env,globalPlanner)
% Function that plots the global plan in the environment

plan = globalPlanner.worldCoordinates;

arc = 0:0.1:2*pi;

figure;
hold all;
drawGlobalPlan(plan);
drawEnv(env.obst,arc);
axis equal;