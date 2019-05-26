function localPlanner = initializeLocalPlanner(localPlanner,veh,log)

    % prints  
    LogicalStr = {'OFF', 'ON'};
    fprintf('\t \t Linear End: \t %s \n', LogicalStr{localPlanner.withLinearEndInitial+1});
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
        % fprintf('\t \t Building problem - may take some time... \n');
        fprintf('\t \t !!! Building still in debug - skipped !!! \n');
    end

%     % setup parametric optimization problem:
%     if localPlanner.rebuildSolver
%         problemIpoptA 	= optim_setup(localPlanner,veh,log,'ipopt',false);
%         problemIpoptB 	= optim_setup(localPlanner,veh,log,'ipopt',true);
%     else
%         load('problemIpoptA.mat');
%         load('problemIpoptB.mat');
%     end
%     localPlanner.solver.problemIpoptA = problemIpoptA;
%     localPlanner.solver.problemIpoptB = problemIpoptB;

    % select most restrictive vehicle dynamic constraints:
    fprintf('\t \t Getting dynamic limits \n');
    localPlanner = getDynamicLimits(localPlanner,veh);

end