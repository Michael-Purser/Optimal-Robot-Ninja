function [I,D] = distanceToRectangleEdge(p,v,C)
% Function that returns the distance to a rectangle along robot rangefinder
% sensor beam.
% Inputs:
%       p       robot position
%       v       vector starting from robot position in direction of
%               sensorbeam, length is sensor horizon
%       C       matrix containing rectangle corner points
% Outputs:
%       I       Boolean indicating if sensor can see rectangle
%       d       If I=1, distance from robot to rectangle along sensor beam,
%               if I=0 returns d=0

% Initialize distance vector:
dist = inf*ones(1,size(C,2));

% Append a column to C for iteration step:
C(:,end+1) = C(:,1);

% Iterate over the vectors:
for k = 1:(size(C,2)-1)
    pR = C(:,k);
    vR = C(:,k)-C(:,k+1);
    [i,d] = distanceToEdge(pR,p,vR,v);
    if i==1
        dist(k) = d;
    end
end
if min(dist)==inf
    D = 0;
    I = 0;
else
    D = min(dist);
    I = 1;
end

end
