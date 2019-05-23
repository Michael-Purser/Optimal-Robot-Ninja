function MPC = transformMeasurements(MPC)
% Function that takes the sensor measurements (in local polar coordinates)
% and the sampled preloaded info (in world coordinates) and prepares them
% to the local cartesian frame for use in the optimization routine

    fprintf('Transforming measurements \n');
    
    state       = MPC.currentState;
    orig        = MPC.obstacleData.meas.localPolar;
    preloaded   = MPC.obstacleData.preloaded;

    % Transform local polar measurements to local cartesian measurements
    transLocal = zeros(size(orig));
    for i=1:size(orig,1)
        theta   = orig(i,1);
        R       = orig(i,2);
        transLocal(i,:) = [-R*sin(theta) R*cos(theta)];
    end
    MPC.obstacleData.meas.localCartesian = transLocal;
    
    % Transform measurements to global cartesian measurements
    T = homTrans(state(3),[state(1);state(2)]);
    transGlobal = zeros(size(orig));
    for i=1:size(orig,1)
       A = T*[transLocal(i,:)';1];
       transGlobal(i,:) = A(1:2);
    end
    MPC.obstacleData.meas.globalCartesian = transGlobal;
    
    % Transform preloaded data to local cartesian measurements
    preloadedLocal = zeros(size(preloaded));
    for i=1:size(preloaded,1)
       A = T\[preloaded(i,:)';1];
       preloadedLocal(i,:) = A(1:2);
    end
    MPC.obstacleData.preloadedLocal = preloadedLocal;
    
end