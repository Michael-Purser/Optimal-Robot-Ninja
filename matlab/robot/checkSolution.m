function sit = checkSolution(sit,veh,it)
% For a given trajectory X, calculate the G-value in every point of the
% trajectory and store it for later examination.

meas = sit.meas{it};
X    = sit.Sol.X{it};
sx   = veh.Optim.sigma_x;
sy   = veh.Optim.sigma_y;

G = zeros(1,size(X,2));
for i=1:size(X,2)
    G(i) = checkPosition(meas,X(:,i),sx,sy);
end

sit.Sol.G{end+1} = G;

end