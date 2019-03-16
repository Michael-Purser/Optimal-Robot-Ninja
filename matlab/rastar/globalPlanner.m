function sit = globalPlanner(sit,veh)

% PARAMETERS:
map       = sit.Temp.gmap;
dx        = sit.Temp.dx;
N         = (size(map,1)-1)/2;
H         = veh.Sensor.horizon;
goal_abs  = sit.localGoals{end}(1:2)';  % in local vehicle coordinates
start_abs = [0,0];                      % in local vehicle coordinates
n         = veh.Optim.n + 1;
% G_hat     = veh.Optim.G_hat;
G_hat     = 0.3*veh.Optim.G_hat;

% IF GOAL INSIDE MAP:
if abs(goal_abs(1))<H && abs(goal_abs(2))<H
    fprintf('   goal is inside map \n');
    
    goals_abs = goal_abs;
    
    % MAKE MAP:
    [map,N]    = addBorder(map,N,1);
    goals      = getIndices(map,N,dx,goals_abs,G_hat);
    start      = getIndices(map,N,dx,start_abs,G_hat);

    % ALGORITHM:
    [path,goal_index] = RAStar(start,goals,map,N,G_hat);
    % correct indices of path:
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
    fprintf('   goal is outside map \n');
    
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
    % correct indices of path:
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

% ADD EXCEPTION:
% If because of 'unique' command above the path_abs only contains 1 entry,
% add the exact starting position:
path_abs = [0 0;path_abs];

% EXPAND PATH UNTIL IT HAS RIGHT LENGTH:
B = interpolateUntil(path_abs(:,1),n);
C = interpolateUntil(path_abs(:,2),n);
path_abs = [B,C];

% ADD PATH TO SITUATION STRUCT:
sit.Init.path{end+1} = path_abs;

end