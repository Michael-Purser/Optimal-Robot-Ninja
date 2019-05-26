function localPlanner = getDynamicLimits(localPlanner,veh)
    % Function that gets the dynamic limits used in the optimization
    % problem from the vehicle info

    localPlanner.params.dynLimits.vel(1)  = min(veh.dynamics.velLimits(1),veh.motors.velLimits(1));
    localPlanner.params.dynLimits.vel(2)  = min(veh.dynamics.velLimits(2),veh.motors.velLimits(2));
    localPlanner.params.dynLimits.acc(1)  = min(veh.dynamics.accLimits(1),veh.motors.accLimits(1));
    localPlanner.params.dynLimits.acc(2)  = min(veh.dynamics.accLimits(2),veh.motors.accLimits(2));
    localPlanner.params.dynLimits.jerk    = veh.dynamics.jerkLimits;
    localPlanner.params.dynLimits.om      = veh.dynamics.omLimits;

end