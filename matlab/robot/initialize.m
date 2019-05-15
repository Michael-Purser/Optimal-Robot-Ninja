function [MPC,env] = initialize(MPC,env,veh)
% initialization function for the MPC loop
    
    % process the knowledge in the environment structure:
    % sort obstacles into mapped and measured and sample the mapped
    % obstacles:
    env = sortObstacles(MPC,env);
    MPC = samplePreloaded(MPC,env);

    % build map representing the discretized known environment used by the 
    % global planner
    % fill the map with the known (mapped) obstacles and inflate the map
    % using gaussians:
    MPC = initializeMap(MPC);
    MPC = fillMap(MPC,2);
    MPC = inflateMap(MPC);

    % initialize the global plan: call the global planner
    MPC = globalPlanner(MPC);

    % testPlot(); % for debug

    % select most restrictive vehicle dynamic constraints:
    MPC = getDynamicLimits(MPC,veh);

    % setup parametric optimization problem:
    if MPC.nav.goalReached == false
        if MPC.nav.rebuild==true
            problemIpopt 	= optim_setup(MPC,veh,'ipopt');
            problemSqp      = optim_setup(MPC,veh,'sqp');
        else
            load('problemIpopt.mat');
            load('problemSqp.mat');
        end
        MPC.nav.problemIpopt = problemIpopt;
        MPC.nav.problemSqp   = problemSqp;
    end

end