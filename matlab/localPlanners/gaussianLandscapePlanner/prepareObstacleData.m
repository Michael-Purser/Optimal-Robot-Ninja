function localPlanner = prepareObstacleData(MPC,localPlanner)
    
    localCartesian  = MPC.obstacleData.meas.localCartesian;
    localGridDx = localPlanner.localGridDx;

    % Process measurements
    localCartesianGrid = processMeas(localCartesian,localGridDx);
    MPC.nav.obstacleData.meas.localCartesianGrid = localCartesianGrid;
    
    % Fill info in variable used by optimization routine
    if localPlanner.withLocalGrid
        localPlanner.obstacleData = localCartesianGrid;
    else
        localPlanner.obstacleData = localCartesian;
    end
end