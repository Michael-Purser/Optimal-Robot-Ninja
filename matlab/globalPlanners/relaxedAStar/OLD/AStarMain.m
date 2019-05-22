% Relaxed A star planner
% Returns found path in robot coordinates to goal in goal list it
% encounters first.

clear;
close all;

% PARAMETERS:
load('Test measurements/test1.mat');
map      = veh.Map.gmap;
dx       = veh.Map.dx;
N        = (size(map,1)-1)/2;
H        = 5;
goals_abs = [ 1, 5;
             -4,-4];    % in robot coordinates
start_abs = [3,-0.2];   % in robot coordinates


% MAKE MAP:
[map,N]    = addBorder(map,N,1);
goals      = getIndices(map,N,dx,goals_abs);
start      = getIndices(map,N,dx,start_abs);


% ALGORITHM:
[path,goal_index] = RAStar(start,goals,map,N);
% remove artificial border and correct indices of start, goal and path:
[map,N] = removeBorder(map,N);
start = start-1;
goals = goals-1;
path  = path-1;


% CONVERT PATH COORDINATES:
% transform the path from grid coordinates to robot coordinates:
path_abs = (path-1)*dx + (dx/2) - H;
% substitute start and goal entry by absolute ones (useful for CasADi
% later):
path_abs(1,:) = start_abs;
path_abs(end,:) = goals_abs(goal_index,:);


% PLOTS:
% time consuming (especially for large N), so disable if you want speed:
plotMap(map,dx,H,start_abs,goals_abs,path_abs);