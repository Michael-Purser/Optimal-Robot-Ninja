function map = addMeasurements(map,n,dx,meas)
% Function that adds the measurements to the local map.
% First the polar sensor measurements are converted to cartesian
% coordinates, then are placed on a map cell following a simple flooring
% strategy.

for k=1:size(meas,1)
    
    % Get cell indices of measurement in map:
    theta   = meas(k,1);
    r       = meas(k,2);
    nx      = n + 1 + floor(-r*sin(theta)/dx);
    ny      = n + 1 + floor(r*cos(theta)/dx);
    
    % Add to map:
    map(nx,ny) = 1;
    
end

end