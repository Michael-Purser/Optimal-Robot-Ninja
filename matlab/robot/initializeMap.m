function MPC = initializeMap(MPC)
    % Function that initializes the map
    
    MPC.nav.map.width   = 20;
    MPC.nav.map.height  = 20;
    MPC.nav.map.dx      = 0.05;
    MPC.nav.map.dy      = 0.05;
    MPC.nav.map.Nw      = ceil(MPC.nav.map.width/MPC.nav.map.dx);
    MPC.nav.map.Nh      = ceil(MPC.nav.map.height/MPC.nav.map.dx);
    % make sure the map always has a center cell by making the cell numbers
    % uneven if they are even:
    if rem(MPC.nav.map.Nw,2)==0
        MPC.nav.map.Nw = MPC.nav.map.Nw + 1;
    end
    if rem(MPC.nav.map.Nh,2)==0
        MPC.nav.map.Nh = MPC.nav.map.Nh + 1;
    end
    % update dx and dy to correct values
    MPC.nav.map.dx = MPC.nav.map.width/MPC.nav.map.Nw;
    MPC.nav.map.dy = MPC.nav.map.height/MPC.nav.map.Nh;
    MPC.nav.map.values  = zeros(MPC.nav.map.Nw,MPC.nav.map.Nh);
    MPC.nav.map.center  = [0;0];

end