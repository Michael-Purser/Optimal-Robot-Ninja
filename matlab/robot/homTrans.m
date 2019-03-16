function T = homTrans(theta,p)
% Function that returns the homogeneous transformation matrix between two
% coordinate systems A and B.
% Usage: p_A = T*p_B
% Inputs:
%       theta       Angle between axes of B and axes of A (zero is on axis
%                   of A)
%       p           Position of origin of B expressed in A

T = [cos(theta) -sin(theta) p(1);
     sin(theta) cos(theta)  p(2);
     0          0           1];

end