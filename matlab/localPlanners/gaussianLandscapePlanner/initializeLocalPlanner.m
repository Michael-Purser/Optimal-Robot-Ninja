function localPlanner = initializeLocalPlanner(localPlanner,veh,log)

    % prints  
    LogicalStr = {'OFF', 'ON'};
    fprintf('\t \t Local grid: \t %s \n', LogicalStr{localPlanner.withLocalGrid+1});
    fprintf('\t \t Rebuild: \t %s \n', LogicalStr{localPlanner.rebuildSolver+1});
    if localPlanner.rebuildSolver
        fprintf('\t \t Constraints: \n');
        fprintf('\t \t * max dist: \t %s \n', ...
            LogicalStr{localPlanner.withMaxDistConstraints+1});
        fprintf('\t \t * vel: \t %s \n', ...
            LogicalStr{localPlanner.withVelocityConstraints+1});
        fprintf('\t \t * pos vel: \t %s \n', ...
            LogicalStr{localPlanner.withPositiveVelocityConstraints+1});
        fprintf('\t \t * omega: \t %s \n', ...
            LogicalStr{localPlanner.withOmegaConstraints+1})
        fprintf('\t \t * accel: \t %s \n', ...
            LogicalStr{localPlanner.withAccelerationConstraints+1});
        fprintf('\t \t * jerk: \t %s \n', ...
            LogicalStr{localPlanner.withJerkConstraints+1});
        fprintf('\n');
        fprintf('\t \t Building problem - may take some time... \n');
    end

    % setup parametric optimization problem:
    if localPlanner.rebuildSolver
        problemIpoptA 	= optim_setup(localPlanner,veh,log,'ipopt',false);
        problemIpoptB 	= optim_setup(localPlanner,veh,log,'ipopt',true);
        problemSqpA     = optim_setup(localPlanner,veh,log,'sqp',false);
        problemSqpB     = optim_setup(localPlanner,veh,log,'sqp',true);
    else
        load('problemIpoptA.mat');
        load('problemIpoptB.mat');
        load('problemSqpA.mat');
        load('problemSqpB.mat');
    end
    localPlanner.solver.problemIpoptA = problemIpoptA;
    localPlanner.solver.problemIpoptB = problemIpoptB;
    localPlanner.solver.problemSqpA   = problemSqpA;
    localPlanner.solver.problemSqpB   = problemSqpB;

    % select most restrictive vehicle dynamic constraints:
    fprintf('\t \t Getting dynamic limits \n');
    localPlanner = getDynamicLimits(localPlanner,veh);

end