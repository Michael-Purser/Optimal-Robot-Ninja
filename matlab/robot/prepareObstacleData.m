function MPC = prepareObstacleData(MPC)
% Function that takes the sensor measurements (in local polar coordinates)
% and the sampled preloaded info (in world coordinates) and prepares them
% to the local cartesian frame for use in the optimization routine
    
    state       = MPC.nav.currentState;
    orig        = MPC.nav.obstacleData.meas.orig;
    preloaded   = MPC.nav.obstacleData.preloaded;

    % Transform local polar measurements to local cartesian measurements
    trans = zeros(size(orig));
    for i=1:size(orig,1)
        theta   = orig(i,1);
        R       = orig(i,2);
        trans(i,:) = [-R*sin(theta) R*cos(theta)];
    end
    MPC.nav.obstacleData.meas.trans = trans;
    
    % Transform preloaded data to local cartesian measurements
    preloadedLocal = zeros(size(preloaded));
    T = homTrans(state(3),[state(1);state(2)]);
    for i=1:size(preloaded,1)
       A = T*[preloaded(i,:)';1];
       preloadedLocal(i,:) = A(1:2);
    end
    
    % Fill info in variable used by optimization routine
    MPC.nav.opt.obst = [trans;preloadedLocal];
end