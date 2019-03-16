function sit = addGaussianToMap(sit,veh)
% Function that takes the local robot map and returns a new map containing
% the values of the G-landscape

map = sit.Temp.map;
h   = veh.Sensor.horizon;
n   = veh.Map.N;
sx  = veh.Optim.sigma_x;
sy  = veh.Optim.sigma_y;

% make the gaussian
% on a grid extending 3 standard deviations in each direction
% factor f converts stand dev from cartesian to grid coords
f  = (2*n+1)/(2*h);
nx = ceil(3*f*sx);
ny = ceil(3*f*sy);
g  = makeGaussian(f*sx,f*sy,nx,ny);

% initialize map storing G-landscape values
% (add border to always be able to add the gaussian)
gmap = zeros(size(map,1)+2*nx,size(map,2)+2*ny);

% add gaussian to the map
for i=1:size(map,1)
    for j=1:size(map,2)
        gmap(i:i+2*nx,j:j+2*ny) = gmap(i:i+2*nx,j:j+2*ny) + map(i,j)*g;
    end
end

% remove border added above
gmap = gmap(nx+1:(end-nx),ny+1:(end-ny));

% add G-map to struct:
sit.Temp.gmap = gmap;

end