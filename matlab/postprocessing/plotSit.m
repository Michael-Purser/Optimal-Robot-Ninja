function plotSit(MPC,env)
% Function that plots the situation: environment with obstacles, global
% start and end goals

globalStart = MPC.nav.globalStart;
globalGoal  = MPC.nav.globalGoal;

arc = 0:0.01:2*pi;

figure;
hold all;
drawEnv(env.obst,arc);
drawStartAndGoal(globalStart,globalGoal);
axis equal;
title('Obstacles in world frame');