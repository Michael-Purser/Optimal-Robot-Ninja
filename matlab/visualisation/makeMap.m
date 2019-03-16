function sit = makeMap(sit,veh)
% Function that makes a local map centered around the robot of height and
% width equal to twice the sensor measuring range.
% The map is represented by a matrix; the value in each cell is zero for
% now, but will be filled with measurement data later.
% The discretization is 2*N+1 samples on each side, to assure the map
% always has an uneven nb of cells and so that the robot can be placed in
% the center.
%
% Note: ROWS are THE X COORDINATE, COLUMNS are THE Y COORDINATE.

h = veh.Sensor.horizon;
n = veh.Map.N;

sit.Temp.map = zeros(2*n+1,2*n+1);
sit.Temp.dx  = 2*h/(2*n+1);

end