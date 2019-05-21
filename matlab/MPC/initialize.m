function [MPC,env] = initialize(MPC,env,veh)
% initialization function for the MPC loop

    % initialize first state
    MPC.nav.currentState    = MPC.nav.globalStart;  % Robot starts at global start
    MPC.nav.currentVelocity = [0;0];                % Robot starts from standstill
    
    % initialize logging variables with first state
    % (only if logging activated)
    if MPC.log.logBool
        MPC.log.states{end+1}     = MPC.nav.currentState;
        MPC.log.velocities{end+1} = [0;0];
    end
    
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
    if MPC.nav.rebuild==true
        problemIpoptA 	= optim_setup(MPC,veh,'ipopt',false);
        problemIpoptB 	= optim_setup(MPC,veh,'ipopt',true);
        problemSqpA     = optim_setup(MPC,veh,'sqp',false);
        problemSqpB     = optim_setup(MPC,veh,'sqp',true);
    else
        load('problemIpoptA.mat');
        load('problemIpoptB.mat');
        load('problemSqpA.mat');
        load('problemSqpB.mat');
    end
    MPC.nav.problemIpoptA = problemIpoptA;
    MPC.nav.problemIpoptB = problemIpoptB;
    MPC.nav.problemSqpA   = problemSqpA;
    MPC.nav.problemSqpB   = problemSqpB;

end