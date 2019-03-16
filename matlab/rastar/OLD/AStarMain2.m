% Relaxed A star planner

% For now please run FIRST TWO SECTIONS '../main.m' before running this, so
% that struct 'veh' and measurements are made.
% (I wanted to save some cases to test the planner with but haven't got
% round to it yet).

% PARAMETERS:
load('Test measurements/test1.mat');
gmap     = veh.Map.gmap;
dx       = veh.Map.dx;
N        = (size(gmap,1)-1)/2;
H        = 5;
goals_abs = [0,5];       % in robot coordinates
start_abs = [3,-0.2];   % in robot coordinates

% MAKE MAP:
map        = zeros(size(gmap,1),size(gmap,2),2);
map(:,:,1) = gmap;
[map,N]    = addBorder(map,N,1);
goals      = getIndices(map,N,dx,goals_abs);
start      = getIndices(map,N,dx,start_abs);
%start      = [N+1;N+1];   % in grid coordinates
                          % robot is always in middle of the map

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
open(4) = euclidianDistance(start,goals);
% initialize iteration counter:
n = 1;
% loop:
while (isempty(open)==0 && map(goals(1),goals(2),2)==inf)
    % get next current cell and remove from open set:
    [current,open] = getRemoveCurrent(open);
    fprintf('current: [%i,%i] \n',current(1),current(2));
    % get the admissible neighbours of current (leave out cells that
    % contain an obstacle) and examin them (give them g and f scores and
    % add them to open if necessary):
    neighbours = getNeighbours(current,map);
    [map,open] = examinNeighbours(current,neighbours,map,goals,open,tBreak);
    % increase iteration counter:
    n = n+1;
end
fprintf('number of iterations/cells examined: %i \n',n);
fprintf('number of cells in map: %i \n',(2*(N+1)+1)^2);
if map(goals(1),goals(2),2)==inf
    error('Failed to find a path to goal');
else
    fprintf('Found a path! \n');
    path = reconstructPath(goals,map);
end
% remove artificial border and correct indices of start, goal and path:
[map,N] = removeBorder(map,N);
start = start-1;
goals = goals-1;
path  = path-1;
% flip the path vector to have it in right order:
path  = flipud(path);


% CONVERT PATH COORDINATES:
% transform the path from grid coordinates to robot coordinates:
path_abs = (path-1)*dx + (dx/2) - H;
% substitute start and goal entry by absolute ones (useful for CasADi
% later):
path_abs(1,:) = start_abs;
path_abs(end,:) = goals_abs;


% PLOTS:
% time consuming (especially for large N), so disable if you want speed:
plotMap(map,dx,H,start_abs,goals_abs,path_abs);