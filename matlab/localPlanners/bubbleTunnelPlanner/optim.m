function localPlanner = optim(localPlanner,veh,MPC_iteration)
% Formulate and solve the minimum time optimization problem.
% Vehicle model: diff-drive.

% parameters:
L           = veh.geometry.wheelBase;
n           = localPlanner.params.horizon;

withLinearEnd           = localPlanner.withLinearEndInitial;
linearEndSwitchDistance = localPlanner.linearEndSwitchDistance;

withMaxDist = localPlanner.withMaxDistConstraints;
withV       = localPlanner.withVelocityConstraints;
withA       = localPlanner.withAccelerationConstraints;
withJ       = localPlanner.withJerkConstraints;
withOm      = localPlanner.withOmegaConstraints;

u_min       = localPlanner.params.dynLimits.vel(1);
u_max       = localPlanner.params.dynLimits.vel(2);
a_min       = localPlanner.params.dynLimits.acc(1);
a_max       = localPlanner.params.dynLimits.acc(2);
j_min       = localPlanner.params.dynLimits.jerk(1);
j_max       = localPlanner.params.dynLimits.jerk(2);
om_min      = localPlanner.params.dynLimits.om(1);
om_max      = localPlanner.params.dynLimits.om(2);
beta        = localPlanner.params.maxDistBeta;
xglobal_x   = localPlanner.params.xglobalx;
xglobal_y   = localPlanner.params.xglobaly;
radii       = localPlanner.params.radii;

solver      = localPlanner.solver.type;

max_meas    = 1000;
max_gplan   = 1000;

% get measurements (in cartesian coordinates):
meas        = localPlanner.obstacleData;
meas        = [meas;20*ones(max_meas-size(meas,1),2)];

% initialize opti:
opti = casadi.Opti();

% initial and final positions + initial guess for time and states:
x_begin = localPlanner.params.start;
x_final = localPlanner.params.goal;
u_begin = localPlanner.params.startVelocity;

% make the max dist value:
maxDist = beta*norm(x_begin(1:2)-x_final(1:2))/n;

% distinction with or without linear end inital guess
if withLinearEnd
    linearSelectorBool = (MPC_iteration==1 || norm(x_final(1:2))<linearEndSwitchDistance);
else
    linearSelectorBool = (MPC_iteration==1);
end

if strcmp(solver,'ipopt')==1
    fprintf('\t \t Using IPOPT solver \n');
    % if first iteration, make initial guesses; else 'warm-start' the
    % solver with previous solution:
    if linearSelectorBool 
        fprintf('\t \t NOT warm-started \n');
        theta_init = linspace(0,x_final(3),n+1);
        x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
            theta_init];
        u_init  = zeros(2,n);
        T_init = norm(x_begin(1:2)-x_final(1:2))/u_max;
    else
        fprintf('\t \t Warm-started \n');
        x_init = localPlanner.sol.x;
        u_init = localPlanner.sol.u;
        T_init = localPlanner.sol.T;
    end
    % select problem to solve depending on wether endgoal is in view or
    % not:
    if localPlanner.goalInView
        fprintf('\t \t Goal in view \n');
        problem    = localPlanner.solver.problemIpoptB;
    else
        fprintf('\t \t Goal not in view \n');
        problem    = localPlanner.solver.problemIpoptA;
    end
    
end

% log the maxDist value
localPlanner.params.maxDist = maxDist;

% log the initial guesses
localPlanner.init.x = x_init;
localPlanner.init.u = u_init;
localPlanner.init.T = T_init;

% try solving problem; if no success, log in localPlanner:

try
    % buildup input list:
    inputCellArray = {x_init,u_init,T_init,L,n,x_begin,x_final,u_begin,xglobal_x,xglobal_y,radii};
    if withV
        inputCellArray{end+1} = u_min;
        inputCellArray{end+1} = u_max;
    end
    if withA
        inputCellArray{end+1} = a_min;
        inputCellArray{end+1} = a_max;
    end
    if withJ
        inputCellArray{end+1} = j_min;
        inputCellArray{end+1} = j_max;
    end
    if withOm
        inputCellArray{end+1} = om_min;
        inputCellArray{end+1} = om_max;
    end
    if withMaxDist
        inputCellArray{end+1} = maxDist;
    end
    
    eval('[X,U,T] = problem(inputCellArray{:});');

    % append to struct:
    localPlanner.sol.x = opti.value(X);
    localPlanner.sol.u = opti.value(U);
    localPlanner.sol.T = opti.value(T);
    localPlanner.sol.stats = getSolverStats(problem);
    localPlanner.sol.success = true;
    
catch ME
    localPlanner.sol.stats = getSolverStats(problem);
    localPlanner.sol.success = false;
    
end

% lamg = sol.value(opti.lam_g);

% append to struct:
% sit.Sol.lamg{end+1} = lamg;

end