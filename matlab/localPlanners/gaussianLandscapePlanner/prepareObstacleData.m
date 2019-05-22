function localPlanner = prepareObstacleData(MPC,localPlanner)
    
    transLocal  = MPC.obstacleData.meas.transLocal;
    localGridDx = localPlanner.localGridDx;

    % Process measurements
    transLocalGrid = processMeas(transLocal,localGridDx);
    MPC.nav.obstacleData.meas.transLocalGrid = transLocalGrid;
    
    % Fill info in variable used by optimization routine
    if localPlanner.withLocalGrid
        localPlanner.obstacleData = transLocalGrid;
    else
        localPlanner.obstacleData = transLocal;
    end
end