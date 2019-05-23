function MPC = fillMap(MPC,selector)
% Function that adds the preloaded data to the map.
% The data is added to the grid map cell following:
%   case 1: a flooring strategy --> gives errors with limit measurements
%   case 2: a 4-quadrant strategy --> more robust

Nw      = MPC.map.Nw;
Nh      = MPC.map.Nh;
dx      = MPC.map.dx;
map     = MPC.map.values;
meas    = [MPC.obstacleData.preloaded;MPC.obstacleData.meas.globalCartesian];
% NOTE: if called before MPC loop, no measurements yet --> it will only
% fill map with preloaded data. This behaviour is used to generate the map
% for the global planner.

switch selector
    
    % case 1: flooring strategy (gives errors sometimes)
    case 1
        for k=1:size(meas,1)
            % get cell indices of measurement in map:
            x       = meas(k,1);
            y       = meas(k,2);
            nx      = ceil(Nw/2) + floor(x/dx);
            ny      = ceil(Nh/2) + ceil(y/dx);
            % add to map:
            map(nx,ny) = 1;
        end
    
    % case 2: 4-quadrant strategy
    case 2
        for k=1:size(meas,1)
            % get cell indices of measurement in map:
            x       = meas(k,1);
            y       = meas(k,2);
            theta   = atan2(y,x);
            if 0<=theta<pi/2
                nx = ceil(Nw/2) + floor(x/dx);
                ny = ceil(Nh/2) + floor(y/dx);
            elseif pi/2<=theta<=pi
                nx = ceil(Nw/2) + ceil(x/dx);
                ny = ceil(Nh/2) + floor(y/dx);
            elseif -pi/2<=theta<0
                nx = ceil(Nw/2) + floor(x/dx);
                ny = ceil(Nh/2) + ceil(y/dx);
            elseif -pi<theta<-pi/2
                nx = ceil(Nw/2) + ceil(x/dx);
                ny = ceil(Nh/2) + ceil(y/dx);
            end
            % add to map:
            map(nx,ny) = 1;
        end

end

MPC.map.values = map;

end