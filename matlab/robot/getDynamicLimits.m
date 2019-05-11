function MPC = getDynamicLimits(MPC,veh)

    MPC.nav.opt.dynamicLimits.vel(1) = min(veh.dynamics.velLimits(1),veh.motors.velLimits(1));
    MPC.nav.opt.dynamicLimits.vel(2) = min(veh.dynamics.velLimits(2),veh.motors.velLimits(2));
    MPC.nav.opt.dynamicLimits.acc(1) = min(veh.dynamics.accLimits(1),veh.motors.accLimits(1));
    MPC.nav.opt.dynamicLimits.acc(2) = min(veh.dynamics.accLimits(2),veh.motors.accLimits(2));
    MPC.nav.opt.dynamicLimits.om     = veh.dynamics.omLimits;

end