function C = rectangleCorners(p,dx,dy,theta)
% Function that returns the positions of the corner points of a rectangle,
% given the center point coordinates, the inclination and the side 
% half-lengths dx and dy.

% in obstacle coordinate frame:
c1 = [-dx dy 1]';
c2 = [dx dy 1]';
c3 = [dx -dy 1]';
c4 = [-dx -dy 1]';

% transform to original frame:
T = homTrans(theta,p);

C1 = T*c1;
C2 = T*c2;
C3 = T*c3;
C4 = T*c4;

C = [C1, C2, C3, C4];

end