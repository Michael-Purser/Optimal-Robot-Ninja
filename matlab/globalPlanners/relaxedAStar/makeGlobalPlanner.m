function globalPlanner = makeGlobalPlanner()

globalPlanner.gridCoordinates  = [];  % Global plan in gridmap coordinates
globalPlanner.worldCoordinates = [];  % Global plan in world coordinates
globalPlanner.lastIndex        = 1;   % Last local goal index along the global plan

globalPlanner.method           = 'RAstar';
globalPlanner.Ghat             = 2;