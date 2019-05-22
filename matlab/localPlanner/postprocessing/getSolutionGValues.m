function log = getSolutionGValues(log)
% function that calculates and stores the G values along each solution
% generated by the local planner

for it=1:size(log.localPlanners,2)

    % get data
    measTransLocal      = log.meas{it}.transLocal;
    measTransLocalGrid  = log.localPlanners{it}.obstacleData;
    X                   = log.localPlanners{it}.sol.x;
    sigma               = log.localPlanners{it}.params.sigma;
    withLocalGrid       = log.localPlanners{it}.withLocalGrid;
    
    % get and log G-values along solutions
    % only do it for grid measurements if the option to use grid measurements
    % has been set to true
    log.localPlanners{it}.post.GValuesOrig    = ...
            checkSolution(measTransLocal,X,sigma);
        
    if withLocalGrid
        log.localPlanners{it}.post.GValuesGrid    = ...
            checkSolution(measTransLocalGrid,X,sigma);
    else
        fprintf(2,'WARNING: variable withLocalGrid set to false, only processing max(G) values for original measurements');
    end
    
end