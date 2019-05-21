function g = makeGaussian(sx,sy,nx,ny)
% Function that returns a gaussian curve defined over a matrix of 2*nx+1
% rows and 2*ny+1 columns.
% The peak of the gaussian is located in the matrix center cell, i.e. in
% (nx+1,ny+1).

g  = zeros(2*nx+1,2*ny+1);
x0 = nx+1;
y0 = ny+1;

for i=1:size(g,1)
    for j=1:size(g,2)
        g(i,j) = exp(-(i-x0)^2/(2*sx^2)-(j-y0)^2/(2*sy^2));
    end
end

end