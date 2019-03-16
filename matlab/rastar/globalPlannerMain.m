% Main of the relaxed A star planner (for testing).

clear;
close all;

% PARAMETERS:
load('Test measurements/test4.mat');
map       = veh.Map.gmap;
dx        = veh.Map.dx;
N         = (size(map,1)-1)/2;
H         = 5;
goal_abs  = [0,10];  % in robot coordinates
start_abs = [4,4];   % in robot coordinates
G_hat     = veh.optim.G_hat;

% IF GOAL INSIDE MAP:
if abs(goal_abs(1))<H && abs(goal_abs(2))<H
    fprintf('Goal is inside map \n');
    
    goals_abs = goal_abs;
    
    % MAKE MAP:
    [map,N]    = addBorder(map,N,1);
    goals      = getIndices(map,N,dx,goals_abs,G_hat);
    start      = getIndices(map,N,dx,start_abs,G_hat);

    % ALGORITHM:
    [path,goal_index] = RAStar(start,goals,map,N,G_hat);
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

% IF GOAL OUTSIDE MAP:
else
    fprintf('Goal is outside map \n');
    
    % MAKE MAP:
    % this time add two borders: an 'escape tunnel' and the obstacle border:
    [map,N]    = addBorder(map,N,0);
    [map,N]    = addBorder(map,N,1);
    start      = getIndices(map,N,dx,start_abs,G_hat);

    % GET GOAL POSITIONS IN MAP:
    goals = getGoals(map,goal_abs,H);
    goals_abs = (goals-2)*dx + (dx/2) - H;

    % ALGORITHM:
    [path,goal_index] = RAStar(start,goals,map,N,G_hat);
    % remove artificial border and correct indices of start, goal and path:
    [map,N] = removeBorder(map,N);
    start = start-2;
    goals = goals-2;
    path  = path-2;
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
    path_abs = unique([path_abs;path_linear(1:end,:)],'rows','stable');
    
end

% PLOTS:
fprintf('Plotting... \n');
% time consuming (especially for large N), so disable if you want speed:
plotMap(map,dx,H,start_abs,goals_abs,path_abs);
fprintf('Done \n');