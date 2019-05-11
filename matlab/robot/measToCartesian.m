function MPC = measToCartesian(MPC)
    % Transform polar measurements to cartesian measurements
    
    orig = MPC.nav.obstacleData.meas.orig;
    
    trans = zeros(size(orig));
    for i=1:size(orig,1)
        theta   = orig(i,1);
        R       = orig(i,2);
        trans(i,:) = [-R*sin(theta) R*cos(theta)];
    end
    
    MPC.nav.obstacleData.meas.trans = trans;

end