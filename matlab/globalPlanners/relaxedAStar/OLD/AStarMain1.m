% Relaxed A star planner

% PARAMETERS:
% Because this all happens in the local robot frame, the start position is
% always in the middle of the map.
N = 25;
H = 5;
meas = veh.Sensor.measurements;
goal_abs = [0,5];         % in absolute coordinates

% MAKE MAP:
[map,dx] = initializeMap(N,H);
[map,N]  = addBorder(map,N);
map      = addMeasurements(map,N,dx,meas);
goal     = getGoal(map,N,dx,goal_abs);
start    = [N+1;N+1];      % in cell coordinates

% ALGORITHM:
% set g score of all cells to inf:
map(:,:,2) = inf;
% make tie-break factor:
tBreak = 1 + (1/(4*N+2));
% initialize open set and add start to it:
open = [start(1), start(2), 0, 0];
% set g-score of start to zero on map:
map(start(1),start(2),2) = 0;
% get f-score of start:
open(4) = euclidianDistance(start,goal);
% initialize iteration counter:
n = 1;
% Loop:
while (isempty(open)==0 && map(goal(1),goal(2),2)==inf)
[current,open] = getRemoveCurrent(open);
fprintf('current: [%i,%i] \n',current(1),current(2));
neighbours = getNeighbours(current,map);
[map,open] = examinNeighbours(current,neighbours,map,goal,open,tBreak);
n = n+1;
end
fprintf('number of iterations/cells examined: %i \n',n);
fprintf('number of cells in map: %i \n',(2*(N+1)+1)^2);
if map(goal(1),goal(2),2)==inf
    error('Failed to find a path to goal');
else
    fprintf('Found a path! \n');
    path = reconstructPath(goal,map);
end
[map,N] = removeBorder(map,N);


% PLOTS:
plotMap(map,dx,H,goal-1,path-1);