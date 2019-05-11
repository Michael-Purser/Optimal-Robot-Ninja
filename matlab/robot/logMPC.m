function MPC = logMPC(MPC)

% function that logs the different aspects of the MPC iteration for future
% use in analysis, displaying, ...
MPC.log.states{end+1}       = MPC.nav.currentState;
MPC.log.velocities{end+1}   = MPC.nav.currentVelocity;
MPC.log.meas{end+1}         = MPC.nav.obstacleData.meas;
% global plans --> TO DO
MPC.log.opts{end+1}         = MPC.nav.opt;
MPC.log.m{end+1}            = MPC.nav.m;
% CPU times --> TO DO

% calculate and log G-values of solution:
MPC.log.opts{end}.sol.Gvalues    = ...
    checkSolution(MPC.nav.obstacleData.meas.trans,MPC.nav.opt.sol.x,MPC.nav.opt.sigma);

end