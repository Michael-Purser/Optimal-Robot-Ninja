function log = logMPC(MPC,localPlanner,globalPlanner,log)

% function that logs the different aspects of the MPC iteration for future
% use in analysis, displaying, ...
log.states{end+1}       = MPC.currentState;
log.velocities{end+1}   = MPC.currentVelocity;
log.meas{end+1}         = MPC.obstacleData.meas;
log.m{end+1}            = MPC.m;
% CPU times --> TO DO

% Log localPlanners, but only log the memory intensive solvers once
if MPC.k == 1
    log.solvers{end+1}  = localPlanner.solver;
end
localPlannerLog             = localPlanner;
localPlannerLog.solvers     = 0;
log.localPlanners{end+1}    = localPlannerLog;

% Only log global plan when needed (save memory):
% (later a more sofisticated strategy can be implemented where the global
% planner is called when the robot is stuck and the logger is called each
% time the global planner is recomputed)
if MPC.k == 1
    log.globalPlanners{end+1}  = globalPlanner;
end

% % calculate and log G-values of solution:
% MPC.log.opts{end}.sol.GValuesOrig    = ...
%     checkSolution(MPC.nav.obstacleData.meas.transLocal,MPC.nav.opt.sol.x,MPC.nav.opt.sigma);
% MPC.log.opts{end}.sol.GValuesGrid    = ...
%     checkSolution(MPC.nav.obstacleData.meas.transLocalGrid,MPC.nav.opt.sol.x,MPC.nav.opt.sigma);

end