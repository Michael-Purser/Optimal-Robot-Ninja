function MPC = getDynamicLimits(MPC,veh)
    % Function that gets the dynamic limits used in the optimization
    % problem from the vehicle info

    MPC.nav.opt.dynLimits.vel(1)  = min(veh.dynamics.velLimits(1),veh.motors.velLimits(1));
    MPC.nav.opt.dynLimits.vel(2)  = min(veh.dynamics.velLimits(2),veh.motors.velLimits(2));
    MPC.nav.opt.dynLimits.acc(1)  = min(veh.dynamics.accLimits(1),veh.motors.accLimits(1));
    MPC.nav.opt.dynLimits.acc(2)  = min(veh.dynamics.accLimits(2),veh.motors.accLimits(2));
    MPC.nav.opt.dynLimits.jerk    = veh.dynamics.jerkLimits;
    MPC.nav.opt.dynLimits.om      = veh.dynamics.omLimits;

end