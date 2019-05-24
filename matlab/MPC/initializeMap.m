function MPC = initializeMap(MPC)
    % Function that initializes the map
    
    MPC.map.width   = 20;
    MPC.map.height  = 20;
    MPC.map.dx      = 0.1;
    MPC.map.dy      = 0.1;
    MPC.map.Nw      = ceil(MPC.map.width/MPC.map.dx);
    MPC.map.Nh      = ceil(MPC.map.height/MPC.map.dx);
    % make sure the map always has a center cell by making the cell numbers
    % uneven if they are even:
    if rem(MPC.map.Nw,2)==0
        MPC.map.Nw = MPC.map.Nw + 1;
    end
    if rem(MPC.map.Nh,2)==0
        MPC.map.Nh = MPC.map.Nh + 1;
    end
    % update dx and dy to correct values
    MPC.map.dx = MPC.map.width/MPC.map.Nw;
    MPC.map.dy = MPC.map.height/MPC.map.Nh;
    MPC.map.values  = zeros(MPC.map.Nw,MPC.map.Nh);
    MPC.map.center  = [0;0];

end