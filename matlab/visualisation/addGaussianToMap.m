function gmap = addGaussianToMap(map,MPC,veh,N)
% Function that takes the local robot map and returns a new map containing
% the values of the G-landscape

h   = veh.sensor.horizon;
sig = MPC.nav.opt.sigma;

% make the gaussian
% on a grid extending 3 standard deviations in each direction
% factor f converts stand dev from cartesian to grid coords
f  = (2*N+1)/(2*h);
nx = ceil(3*f*sig);
ny = ceil(3*f*sig);
g  = makeGaussian(f*sig,f*sig,nx,ny);

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

end