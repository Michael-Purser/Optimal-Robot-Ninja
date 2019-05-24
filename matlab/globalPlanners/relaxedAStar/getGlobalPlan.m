function globalPlanner = getGlobalPlan(MPC,globalPlanner)

% PARAMETERS:
values    = MPC.map.inflated;
dx        = MPC.map.dx;
N         = (MPC.map.Nw-1)/2;
H         = (MPC.map.width)/2;
startAbs  = MPC.globalStart(1:2)';  % in global coordinates
goalAbs   = MPC.globalGoal(1:2)';   % in global coordinates
Ghat      = 0.01*globalPlanner.Ghat;
smooth    = globalPlanner.smooth;   % indicates if the plan has to be smoothed

% IF GOAL INSIDE MAP:
if abs(goalAbs(1))<H && abs(goalAbs(2))<H
    fprintf('\t \t goal is inside map \n');
    
    goalsAbs = goalAbs;
    
    % MAKE MAP:
    [values,N] = addBorder(values,N,1);
    goals      = getIndices(values,N,dx,goalsAbs,Ghat);
    start      = getIndices(values,N,dx,startAbs,Ghat);

    % ALGORITHM:
    [plan,goal_index] = RAStar(start,goals,values,N,Ghat);
    % correct indices of path:
    plan  = plan-1;
    
    % CONVERT PATH COORDINATES:
    % transform the path from grid coordinates to robot coordinates:
    planAbs = (plan-1)*dx + (dx/2) - H;
    % substitute start and goal entry by absolute ones:
    planAbs(1,:) = startAbs;
    planAbs(end,:) = goalsAbs(goal_index,:);

% IF GOAL OUTSIDE MAP:
else
    fprintf('\t \t goal is outside map \n');
    
    % MAKE MAP:
    % this time add two borders: an 'escape tunnel' and the obstacle border:
    [values,N]    = addBorder(values,N,0);
    [values,N]    = addBorder(values,N,1);
    start      = getIndices(values,N,dx,startAbs,Ghat);

    % GET GOAL POSITIONS IN MAP:
    goals = getGoals(values,goalAbs,H);
    goalsAbs = (goals-2)*dx + (dx/2) - H;

    % ALGORITHM:
    [plan,goal_index] = RAStar(start,goals,values,N,Ghat);
    % correct indices of path:
    plan  = plan-2;
    % Get reached goal on map border:
    goal_border_abs = goalsAbs(goal_index(1),:);
    
    % CONVERT PATH COORDINATES:
    % transform the path from grid coordinates to robot coordinates:
    planAbs = (plan-1)*dx + (dx/2) - H;
    % substitute start and goal entry by absolute ones (useful for CasADi
    % later):
    planAbs(1,:) = startAbs;
    planAbs(end,:) = goal_border_abs;
    
    % ADD LINEAR PATH TO ACTUAL GOAL:
    n1 = abs(goal_border_abs(1)-goalAbs(1))/dx;
    n2 = abs(goal_border_abs(2)-goalAbs(2))/dx;
    path_linear = [linspace(goal_border_abs(1),goalAbs(1),max(n1,n2)); ...
        linspace(goal_border_abs(2),goalAbs(2),max(n1,n2))]';
    planAbs = unique([planAbs;path_linear(1:end,:)],'rows','stable');
    
end

% ADD EXCEPTION:
% If because of 'unique' command above the path_abs only contains 1 entry,
% add the exact starting position:
planAbs = [startAbs(1:2);planAbs];

% EXPAND PATH UNTIL IT HAS RIGHT LENGTH:
% B = interpolateUntil(path_abs(:,1),n);
% C = interpolateUntil(path_abs(:,2),n);
% path_abs = [B,C];

% IF DESIRED, SMOOTH THE PLAN USING A SAVITZKY-GOLAY FILTER:
% Values are hardcoded -- may be good to make them variable in the future
if smooth
    fprintf('\t \t smoothing is ON: smoothing the path using Savitzky-Golay filter \n');
    plan2Abs = planAbs;
    plan2Abs(:,1) = sgolayfilt(planAbs(:,1),3,9); % smooth x-component
    plan2Abs(:,2) = sgolayfilt(planAbs(:,2),3,9); % smooth y-component
    planAbs = plan2Abs;
else
    fprintf('\t \t smoothing is OFF \n');
end

% ADD PATH TO SITUATION STRUCT:
globalPlanner.gridCoordinates = plan;
globalPlanner.worldCoordinates = planAbs;

end