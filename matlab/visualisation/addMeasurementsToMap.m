function map = addMeasurementsToMap(map,dx,meas,phi,n,selector)
% Function that adds the measurements to the local map.
% The polar rangefinder measurements are converted to cartesian
% coordinates, then are placed on a grid map cell following:
%   case 1: a flooring strategy --> gives errors with limit measurements
%   case 2: a 4-quadrant strategy --> more robust

switch selector
    
    % case 1: flooring strategy (gives errors sometimes)
    case 1
        for k=1:size(meas,1)
            % get cell indices of measurement in map:
            theta   = meas(k,1)-phi;
            r       = meas(k,2);
            nx      = n + 1 + floor(-r*sin(theta)/dx);
            ny      = n + 1 + ceil(r*cos(theta)/dx);
            % add to map:
            map(nx,ny) = 1;
        end
    
    % case 2: 4-quadrant strategy
    case 2
        for k=1:size(meas,1)
            % get cell indices of measurement in map:
            theta   = meas(k,1)+phi;
            r       = meas(k,2);
            if 0<=rem(theta,2*pi)<pi/2
                nx = n + 1 + ceil(-r*sin(theta)/dx);
                ny = n + 1 + floor(r*cos(theta)/dx);
            elseif pi/2<=rem(theta,2*pi)<pi
                nx = n + 1 + ceil(-r*sin(theta)/dx);
                ny = n + 1 + ceil(r*cos(theta)/dx);
            elseif -pi/2<=rem(theta,2*pi)<0
                nx = n + 1 + floor(-r*sin(theta)/dx);
                ny = n + 1 + floor(r*cos(theta)/dx);
            elseif -pi<=rem(theta,2*pi)<-pi/2
                nx = n + 1 + floor(-r*sin(theta)/dx);
                ny = n + 1 + ceil(r*cos(theta)/dx);
            end
            % add to map:
            map(nx,ny) = 1;

        end

end