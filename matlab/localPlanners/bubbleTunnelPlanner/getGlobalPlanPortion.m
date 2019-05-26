function localPlanner = getGlobalPlanPortion(MPC,globalPlanner,localPlanner)
    
    currentState = MPC.currentState;
    xglobalx = globalPlanner.worldCoordinates(1:globalPlanner.lastIndex,1);
    xglobaly = globalPlanner.worldCoordinates(1:globalPlanner.lastIndex,2);
    
    % transform to local frame before calling solver:
    Xglobal = toLocalFrame(MPC.currentState(1:2),MPC.currentState(3),[xglobalx';xglobaly']);
    
    % replace first element by 0;
    % the first element will typically slightly deviate from 0 due to map
    % discretization; but it is best of the local planner gets a list with
    % exactly zeros as a start, therefore replace first element.
    localPlanner.params.xglobalx = [0;Xglobal(1,2:end)']; 
    localPlanner.params.xglobaly = [0;Xglobal(2,2:end)'];
    
end