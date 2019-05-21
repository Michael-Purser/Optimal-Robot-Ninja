function g = gaussianValue(x,p,sx,sy)
% Returns the value of a gaussian bell-curve in a given point.
% Inputs:
%       x       gaussian curve center point
%       p       point in which value has to be calculated
%       sx      gaussian standard deviation along x
%       sy      gaussian standard deviation along y
% Output:
%       g       gaussian value at p

g = exp(-(x(:,1)-p(1)).^2./(2*sx^2)-(x(:,2)-p(2)).^2./(2*sy^2));

end