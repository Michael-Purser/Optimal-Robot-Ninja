function [d,P] = distanceToRectangle(p,pR,dx,dy,theta)
% Function that returns the MINIMAL distance from the robot to a rectangle,
% and the coordinates of the closest point on the rectangle to the robot.
% Inputs:
%       p       robot position
%       pR      rectangle center position
%       dx      half width of rectangle
%       dy      half height of rectangle
%       theta   rectangle inclination w.r.t. horizontal
% Outputs:
%       d       minimal distance from robot to rectangle
%       P       coordinates of closest point to robot on rectangle edge

% transform robot position to obstacle frame - easier for calculations:
T = homTrans(theta,pR);
p = T\p;
px = abs(p(1));
py = abs(p(2));

% throw error if robot is inside rectangle:
if (px<dx && py<dy)
    error('robot appears to be inside rectangular obstacle');
end

% if robot outside "grey zone" (see handwritten notes), closest point is
% corner point:
if (px>=dx && py>=dy)
    d = sqrt((px-dx)^2 + (py-dy)^2);
    P = [sign(p(1))*dx;sign(p(2))*dy;1];

% if robot inside "grey zone":
else
    if px>dx
        d = px-dx;
        P = [sign(p(1))*dx;p(2);1];
    else
        d = py-dy;
        P = [p(1);sign(p(2))*dy;1];
    end
end

% transform location closest point back to initial frame of reference:
P = T*P;

end