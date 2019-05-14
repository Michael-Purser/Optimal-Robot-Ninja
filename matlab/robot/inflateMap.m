function MPC = inflateMap(MPC)

values = MPC.nav.map.values;
N         = (MPC.nav.map.Nw-1)/2;
h = (MPC.nav.map.width)/2;
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
gmap = zeros(size(values,1)+2*nx,size(values,2)+2*ny);

% add gaussian to the map
for i=1:size(values,1)
    for j=1:size(values,2)
        gmap(i:i+2*nx,j:j+2*ny) = gmap(i:i+2*nx,j:j+2*ny) + values(i,j)*g;
    end
end

% remove border added above
MPC.nav.map.inflated = gmap(nx+1:(end-nx),ny+1:(end-ny));


end