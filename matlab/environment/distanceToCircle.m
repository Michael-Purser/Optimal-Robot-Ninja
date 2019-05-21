function [d,P] = distanceToCircle(p,pC,R)
% Function that returns the shortest geometric distance from the circle
% edge to the vehicle.
% Inputs:
%       p       vehicle position
%       pC      circle center position
%       R       circle radius
% Outputs:
%       d       distance from robot to circle edge
%       P       coordinates of the closest point to the robot on circle
%               edge

d = norm(p-pC) - R;
P = p + (pC-p)*d/norm(pC-p);

% throw error if vehicle is inside circle:
if d<0
    error('vehicle appears to be inside circular obstacle');
end

end