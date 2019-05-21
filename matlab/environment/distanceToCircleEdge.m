function [I,d] = distanceToCircleEdge(v1,v2,R)
% Function that returns the distance measured by the rangefinder to a
% circle edge.
% Inputs:
%       v1      vector from robot position to circle centerpoint
%       v2      vector starting from robot position in direction of
%               sensorbeam, length is sensor horizon
%       R       circle radius
% Outputs:
%       I       Boolean indicating if v1 and v2 intersect
%       d       If I=1, distance from robot to circle edge along sensor
%               beam, if I=0 returns d=0
% For details on calculations, see handwritten notes.

I = 1;

% check _infinite_ sensor beam intersects the circle:
theta = asin(norm(cross(v1,v2))/(norm(v1)*norm(v2)));
theta_max = asin(R/norm(v1));
if (abs(theta)>theta_max || v1'*v2<0)
    I = 0;
    d = 0;
end

% calculate distance:
if I==1
   nu = pi - asin(sin(theta)*norm(v1)/R);
   % make exception for case nu = pi:
   if abs(pi-nu)<1e-8
       d = abs(norm(v1)-R);
   else
       delta = pi - theta - nu;
       d = abs(sin(delta)*R/sin(theta));
   end
   % if measured distance too far, return no intersection:
   if d>norm(v2)
       I = 0;
       d = 0;
   end
end

end