function sit = addPathToMap(sit,veh)

map     = sit.Temp.map;
dx      = sit.Temp.dx;
X       = sit.Sol.X{end};
n       = veh.Map.N;

map     = zeros(size(map));

for k=1:size(X,2)
    
    % get cell indices of measurement in map:
    ix      = n + 1 + floor(X(1,k)/dx);
    iy      = n + 1 + floor(X(2,k)/dx);
    
    % add to map:
    map(ix,iy) = 10;
    
end

sit.Temp.map = map;

end