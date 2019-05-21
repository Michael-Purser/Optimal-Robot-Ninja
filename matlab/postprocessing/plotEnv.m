function plotEnv(env)
% Function that plots the obstacles in the environment, both known and
% unknown

arc = 0:0.01:2*pi;

figure;
hold all;
drawEnv(env.obst,arc);
axis equal;
title('Obstacles in world frame');