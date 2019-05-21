function MPC = getStatesToAdvance(MPC,veh,localPlanner,nSelector)
% Function that calculates how many states to move the vehicle between
% successive MPC iterations.
% Different strategies are implemented for comparison.

fprintf('Getting number of states to actuate robot \n');

switch nSelector
    
    % Strategy 1: Based on distance to target (old)
    case 1
        n_original = MPC.opt.horizon;
        n_min      = ceil(veh.Optim.n/10);
        n_max      = ceil(veh.Optim.n/10);
        n_new = ceil(n_original*(1-norm([MPC.nav.currentState(1:2);1]-[MPC.nav.globalGoal(1:2);1])/norm([MPC.nav.globalStart(1:2);1]-[MPC.nav.globalGoal(1:2);1])));
        if n_new<n_min
            n_new = n_min;
        elseif n_new>n_max
            n_new = n_max;
        end
    
    % Strategy 2: based on actuation frequency
    case 2
        f_min   = veh.motors.fmax; % Hz
        if localPlanner.sol.T == 0
            T_n = localPlanner.init.T;
        else
            T_n = localPlanner.sol.T(end);
        end
        n = localPlanner.params.horizon;
        n_new = ceil(n/(T_n*f_min));
        if n_new>n
            n_new = n;
        end
        
    % Strategy 3: same as 2, but with exact change (instead of ceil)
    case 3
        f_min   = veh.motors.fmax; % Hz
        if localPlanner.sol.T == 0
            T_n = localPlanner.init.T;
        else
            T_n = localPlanner.sol.T(end);
        end
        n = localPlanner.params.horizon;
        n_new = 1+n/(T_n*f_min); % +1 because of matlab indexing!!
        if n_new>n
            n_new = n;
        end
end

MPC.m            = n_new;