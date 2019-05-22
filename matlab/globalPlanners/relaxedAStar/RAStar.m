function [path,goal_index] = RAStar(start,goals,gmap,N,G_hat)
% Relaxed A star planner
% For an input map, a start cell in that map, and a set of goal cells in
% that map, returns a path in grid coordinates starting from the start cel
% to the first goal cell encountered.
% DON'T FORGET TO POST-PROCESS FOUND PATH!! (e.g. index correction due to
% preprocessing of map).

% ADD LAYER TO MAP FOR G-SCORES:
map        = zeros(size(gmap,1),size(gmap,2),2);
map(:,:,1) = gmap;

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
open(4) = norm(start-goals);
% initialize iteration counter:
n = 1;
% loop:
goals_reached = [];
while (isempty(open)==0 && isempty(goals_reached))
    % get next current cell and remove from open set:
    [current,open] = getRemoveCurrent(open);
    % fprintf('current: [%i,%i] \n',current(1),current(2));
    % get the admissible neighbours of current (leave out cells that
    % contain an obstacle) and examin them (give them g and f scores and
    % add them to open if necessary):
    neighbours = getNeighbours(current,map,G_hat);
    [goals_reached,goal_index,~] = intersect(goals,neighbours,'rows');
    [map,open] = examinNeighbours(current,neighbours,map,goals,open,tBreak);
    % increase iteration counter:
    n = n+1;
end
fprintf('\t \t number of iterations/cells examined: %i \n',n);
fprintf('\t \t number of cells in map: %i \n',(2*(N+1)+1)^2);
if isempty(goals_reached)==1
    error('\t \t failed to find a path to goal');
else
    fprintf('\t \t found a path! \n');
    path = reconstructPath(goals(goal_index(1),:),map,G_hat);
end

% flip the path vector to have it in right order:
path  = flipud(path);
