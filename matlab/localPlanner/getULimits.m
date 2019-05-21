function [dusqmax,samax] = getULimits(veh,alpha)
% Function that calculates the limits on the actuator velocities and
% accelerations, given vehicle geometric parameters.
%
% Inputs:
%       veh             struct containing vehicle data
%       alpha           safety factor on the limits: 0<alpha<1
%
% Outputs:
%       dusqmax         'delta u square max': maximum allowed value for the
%                       difference of the squares of u_left and u_right
%       samax           'sum a max': maximum allowed combined acceleration
%                       from both actuators

m = veh.mass;
L = veh.wheelBase;
l = veh.wheelBase;
h = veh.cogHeight;

g = 9.81;

dusqmax = alpha*g*(L^2)*(1/h);
samax   = alpha*g*l*(1/h);

end