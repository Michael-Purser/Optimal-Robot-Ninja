function [map,dx] = initializeMap(N,H)
% Function that makes a local map centered around the robot of height and
% width equal to twice the sensor measuring range.
% The map is represented by a matrix; the value in each cell is zero for
% now, but will be filled with measurement data later.
% The discretization is 2*N+1 samples on each side, to assure the map
% always has an uneven nb of cells and so that the robot can be placed in
% the center.
% A second layer is added to store the g-scores of each cell.
%
% Note: ROWS are THE X COORDINATE, COLUMNS are THE Y COORDINATE.

map = zeros(2*N+1,2*N+1,2);
dx  = 2*H/(2*N+1);

end