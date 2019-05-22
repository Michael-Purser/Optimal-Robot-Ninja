function [MPC,env,globalPlanner,localPlanner,log] = initialize(MPC,env)
% initialization function for the MPC loop  
    
    fprintf('Initializing MPC loop \n');
    
    % make the remaining structs
    fprintf('Making global planner struct \n');
    globalPlanner = makeGlobalPlanner();
    fprintf('Making local planner struct \n');
    localPlanner = makeLocalPlanner();
    fprintf('Making log struct \n');
    log = makeLog();

    % initialize first state
    fprintf('\t Setting first state \n');
    MPC.currentState    = MPC.globalStart;  % Robot starts at global start
    MPC.currentVelocity = [0;0];            % Robot starts from standstill
    
    % process the knowledge in the environment structure:
    % sort obstacles into mapped and measured and sample the mapped
    % obstacles:
    fprintf('\t Sorting environment knowledge \n');
    env = sortObstacles(MPC,env);
    MPC = samplePreloaded(MPC,env);

    % build map representing the discretized known environment used by the 
    % global planner
    % fill the map with the known (mapped) obstacles and inflate the map
    % using gaussians:
    fprintf('\t Initializing map \n');
    MPC = initializeMap(MPC);
    MPC = fillMap(MPC,2);
    MPC = inflateMap(MPC);

    % initialize the global plan: call the global planner
    fprintf('\t Getting global plan \n');
    globalPlanner = getGlobalPlan(MPC,globalPlanner);

    % testPlot(); % for debug

end