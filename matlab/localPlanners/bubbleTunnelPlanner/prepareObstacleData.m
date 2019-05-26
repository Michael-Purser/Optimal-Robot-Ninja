function localPlanner = prepareObstacleData(MPC,localPlanner)
    
    localCartesian  = MPC.obstacleData.meas.localCartesian;
    
    localPlanner.obstacleData = localCartesian;
    
end