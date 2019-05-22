function localPlanner = makeLocalPlanner()

localPlanner.obstacleData  = [];            % Measurement data in cartesian coordinates
localPlanner.localGridDx   = 0.05;          % Local grid cell width

localPlanner.params.start         = [];     % Starting state
localPlanner.params.goal          = [];     % Goal state
localPlanner.params.startVelocity = [];     % Goal state
localPlanner.params.horizon       = 200;    % Problem horizon
localPlanner.params.dynLimits.vel = [];     % Velocity limits
localPlanner.params.dynLimits.acc = [];     % Acceleration limits
localPlanner.params.dynLimits.jerk = [];    % Jerk limits
localPlanner.params.dynLimits.om  = [];     % Angular velocity limits
localPlanner.params.sigma         = 0.1;    % Gaussian landscape standard deviation
localPlanner.params.Ghat          = 0.15;    % Gaussian landscape limit level for vehicle
localPlanner.params.maxDist       = 0;      % Max distance bewteen two solution states
localPlanner.params.maxDistBeta   = 3;      % Used in calculation of above max dist
localPlanner.params.globalPlanR   = 2;      % Following radius for global plan

localPlanner.withLocalGrid        = true;
localPlanner.goalInView           = false;    % Indicate if final goal is within view or not

localPlanner.init.x        = [];
localPlanner.init.u        = [];
localPlanner.init.T        = 0;

localPlanner.sol.x         = [];
localPlanner.sol.u         = [];
localPlanner.sol.T         = 0;
localPlanner.sol.stats     = 0;
localPlanner.sol.success   = false;

localPlanner.solver.type            = 'ipopt';
localPlanner.solver.rebuild         = true; 	% Indicate wether to rebuild (true) or load (false) the problem
localPlanner.solver.problemIpoptA   = 0;    
localPlanner.solver.problemIpoptB   = 0;
localPlanner.solver.problemSqpA     = 0;
localPlanner.solver.problemSqpB     = 0;