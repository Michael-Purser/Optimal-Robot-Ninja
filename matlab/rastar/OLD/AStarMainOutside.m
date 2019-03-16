% Relaxed A star planner
% For case when goal is outside of map.

clear;
close all;


% PARAMETERS:
load('Test measurements/test1.mat');
map      = veh.Map.gmap;
dx       = veh.Map.dx;
N        = (size(map,1)-1)/2;
H        = 5;
goal_abs = [4,7];   % in robot coordinates
start_abs = [3,-0.2];  % in robot coordinates


% MAKE MAP:
% this time add two borders: an 'escape tunnel' and the obstacle border:
[map,N]    = addBorder(map,N,0);
[map,N]    = addBorder(map,N,1);
start      = getIndices(map,N,dx,start_abs);


% GET GOAL POSITIONS:
goals = getGoals(map,goal_abs,H);
goals_abs = (goals-1)*dx + (dx/2) - H;


% ALGORITHM:
[path,goal_index] = RAStar(start,goals,map,N);
% remove artificial border and correct indices of start, goal and path:
[map,N] = removeBorder(map,N);
start = start-1;
goals = goals-1;
path  = path-1;
% Get reached goal on map border:
goal_border_abs = goals_abs(goal_index(1),:);


% CONVERT PATH COORDINATES:
% transform the path from grid coordinates to robot coordinates:
path_abs = (path-1)*dx + (dx/2) - H;
% substitute start and goal entry by absolute ones (useful for CasADi
% later):
path_abs(1,:) = start_abs;
path_abs(end,:) = goal_border_abs;


% ADD LINEAR PATH TO ACTUAL GOAL:
n1 = abs(goal_border_abs(1)-goal_abs(1))/dx;
n2 = abs(goal_border_abs(2)-goal_abs(2))/dx;
path_linear = [linspace(goal_border_abs(1),goal_abs(1),max(n1,n2)); ...
    linspace(goal_border_abs(2),goal_abs(2),max(n1,n2))]';
path_abs = [path_abs;path_linear];


% PLOTS:
% time consuming (especially for large N), so disable if you want speed:
% [map,N] = removeBorder(map,N);
% goals = goals-2;
plotMap(map,dx,H,start_abs,goals_abs,path_abs);