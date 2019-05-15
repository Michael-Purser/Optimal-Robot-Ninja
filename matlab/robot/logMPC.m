function MPC = logMPC(MPC)

% function that logs the different aspects of the MPC iteration for future
% use in analysis, displaying, ...
MPC.log.states{end+1}       = MPC.nav.currentState;
MPC.log.velocities{end+1}   = MPC.nav.currentVelocity;
MPC.log.meas{end+1}         = MPC.nav.obstacleData.meas;
MPC.log.opts{end+1}         = MPC.nav.opt;
MPC.log.m{end+1}            = MPC.nav.m;
% CPU times --> TO DO

% Only log global plan when needed (save memory):
% (later a more sofisticated strategy can be implemented where the global
% planner is called when the robot is stuck and the logger is called each
% time the global planner is recomputed)
if MPC.nav.k == 1
    MPC.log.globalPlans{end+1}  = MPC.nav.globalPlan;
end

% calculate and log G-values of solution:
MPC.log.opts{end}.sol.Gvalues    = ...
    checkSolution(MPC.nav.obstacleData.meas.transLocal,MPC.nav.opt.sol.x,MPC.nav.opt.sigma);

end