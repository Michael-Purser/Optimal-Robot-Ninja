function log = logMPC(MPC,localPlanner,globalPlanner,log)
% function that logs the different aspects of the MPC iteration for future
% use in analysis, displaying, ...

fprintf('Logging is ON: logging MPC iteration \n');

% First iteration logs
if MPC.k == 1
    fprintf('\t logging first iteration \n')
    log.globalStart       = MPC.globalStart;
    log.globalGoal        = MPC.globalGoal;
    log.states{end+1}     = MPC.currentState;
    log.velocities{end+1} = [0;0];
    log.goalTolerance     = MPC.goalTolerance;
    log.preload           = MPC.preload;
    
end

log.states{end+1}       = MPC.currentState;
log.velocities{end+1}   = MPC.currentVelocity;
log.meas{end+1}         = MPC.obstacleData.meas;
log.maps{end+1}         = MPC.map;
log.m{end+1}            = MPC.m;

% Only log global plan when needed (save memory):
% (later a more sofisticated strategy can be implemented where the global
% planner is called when the robot is stuck and the logger is called each
% time the global planner is recomputed)
if MPC.k == 1
    log.globalPlanners{end+1}  = globalPlanner;
end

% Log localPlanners, but only log the memory intensive solvers once
if MPC.k == 1
    log.solvers{end+1}  = localPlanner.solver;
end
localPlannerLog             = localPlanner;
localPlannerLog.solvers     = 0;
log.localPlanners{end+1}    = localPlannerLog;

end