function log = makeLog()

log.logBool         = true;     % if true, log previous solutions, states, ... (including CPU times).
log.exportBool      = false;    % if true, export casadi problem to a .casadi file
log.globalStart     = [];
log.globalGoal      = [];
log.goalTolerance   = 0;
log.preload         = false;
log.states          = {};
log.velocities      = {};
log.meas            = {};
log.maps            = {};
log.globalPlanners  = {};
log.localPlanners   = {};
log.solvers         = {};       % solvers logged separately for memory efficiency
log.m               = {};