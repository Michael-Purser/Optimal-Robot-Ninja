function inForbidden = calc_inForbiddenZone(sit,veh)
% Function that checks the current vehicle position is not in an obstacle.

pos = [sit.states(1:2,:,end);1];
meas = sit.meas{end};
sigma_x = veh.Optim.sigma_x;
sigma_y = veh.Optim.sigma_y;
G_hat = veh.Optim.G_hat;

inForbidden = 0;
G = 0;

for k = 1:size(meas,1)
    th  = meas(k,1);
    r   = meas(k,2);
    p   = [-r*sin(th);r*cos(th)];
    g   = getGaussianValue(pos,p,sigma_x,sigma_y);
    G = G + g;
end

if G>=G_hat
    inForbidden = 1;
end

end