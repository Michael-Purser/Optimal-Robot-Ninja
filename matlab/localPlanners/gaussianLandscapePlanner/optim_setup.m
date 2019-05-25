function problem = optim_setup(localPlanner,veh,log,solver_str,with_end)

% initialize variables:
opti = casadi.Opti();

max_meas    = 1000;
n           = localPlanner.params.horizon;
withMaxDist = localPlanner.withMaxDistConstraints;
withV       = localPlanner.withVelocityConstraints;
withVPos    = localPlanner.withPositiveVelocityConstraints;
withA       = localPlanner.withAccelerationConstraints;
withJ       = localPlanner.withJerkConstraints;
withOm      = localPlanner.withOmegaConstraints;
L           = veh.geometry.wheelBase;

Lp          = opti.parameter(1,1);
np          = opti.parameter(1,1);
uminp       = opti.parameter(1,1);
umaxp       = opti.parameter(1,1);
aminp       = opti.parameter(1,1);
amaxp       = opti.parameter(1,1);
jminp       = opti.parameter(1,1);
jmaxp       = opti.parameter(1,1);
omminp      = opti.parameter(1,1);
ommaxp      = opti.parameter(1,1);
sigmap      = opti.parameter(1,1);
Ghatp       = opti.parameter(1,1);
maxDistp    = opti.parameter(1,1);
measp       = opti.parameter(max_meas,2);
xbeginp     = opti.parameter(3,1);
xfinalp     = opti.parameter(3,1);
ubeginp     = opti.parameter(2,1);

% ode:
ode  = @(x,u)[0.5*(u(1)+u(2))*sin(x(3)); 0.5*(u(1)+u(2))*cos(x(3)); (2/L)*(u(2)-u(1))];

% states integration:
h  = casadi.SX.sym('h');
x  = casadi.SX.sym('x',3);
u  = casadi.SX.sym('u',2);
k1 = ode(x,       u); 
k2 = ode(x+h/2*k1,u);
k3 = ode(x+h/2*k2,u);
k4 = ode(x+h*k3,  u);
xf = x+ h/6 * (k1 + 2*k2 + 2*k3 + k4);
F  = casadi.Function('F',{x,u,h},{xf});

% decision variables
x    = opti.variable(3,n+1);
u    = opti.variable(2,n);
T    = opti.variable(1,n);

%% *** CONSTRAINTS ***

% multiple-shooting constraint
opti.subject_to(F(x(:,1:end-1),u,T./np)==x(:,2:end));

% state constraints
opti.subject_to(x(:,1)==xbeginp);
opti.subject_to(x(:,end)==xfinalp);
if withMaxDist
    opti.subject_to(-maxDistp<=diff(x(1,:))<=maxDistp);
    opti.subject_to(-maxDistp<=diff(x(2,:))<=maxDistp);
end

% velocity constraints
if withV
    opti.subject_to(uminp <= u(1,:) <= umaxp);
    opti.subject_to(uminp <= u(2,:) <= umaxp);
    opti.subject_to(u(:,1) == ubeginp);
end
if withVPos
    opti.subject_to(u(1,:)+u(2,:) >= 0);
end

% end velocity constraint; depends on wether the end goal is in view or
% not:
if with_end
	opti.subject_to(u(:,end) == [0;0]);
end

% platform dynamics constraint (optional):
% [dusqmax,samax] = getULimits(veh,0.8);
% opti.subject_to(abs(u(1,:).^2-u(2,:).^2) <= dusqmax);

% acceleration constraints
if withA
    opti.subject_to(aminp <= np./T(2:end).*u(1,2:end)-np./T(1:end-1).*u(1,1:end-1) <= amaxp);
    opti.subject_to(aminp <= np./T(2:end).*u(2,2:end)-np./T(1:end-1).*u(2,1:end-1) <= amaxp);
end

% jerk constraints; implemented using a three-point stencil second
% derivative scheme
if withJ
    opti.subject_to(jminp <= ((np./T(3:end)).^2).*u(1,3:end)-2*((np./T(2:end-1)).^2).*u(1,2:end-1)+...
        ((np./T(1:end-2)).^2).*u(1,1:end-2) <= jmaxp);
    opti.subject_to(jminp <= ((np./T(3:end)).^2).*u(2,3:end)-2*((np./T(2:end-1)).^2).*u(2,2:end-1)+...
        ((np./T(1:end-2)).^2).*u(2,1:end-2) <= jmaxp);
end

% angular velocity constraint
if withOm
    opti.subject_to(omminp <= (1/Lp)*(u(2,:)-u(1,:)) <= ommaxp);
end

% time constraints
opti.subject_to(T >= 0);
opti.subject_to(T(2:end)==T(1:end-1));

% obstacle avoidance constraint
pos     = casadi.MX.sym('pos',2);
g       = gaussianValue(measp,pos,sigmap,sigmap);
costf   = casadi.Function('costf',{pos},{sum(g)});
opti.subject_to(costf(x(1:2,:))<=Ghatp);


%% *** OBJECTIVE AND SOLVER ***

% objective:
opti.minimize(sum(T)/np);

% solver:
if strcmp(solver_str,'ipopt')==1
    options.ipopt.print_level = 0;
    opti.solver('ipopt',options);
else
    opts = struct;
    opts.convexify_strategy = 'eigen-reflect';
    opts.verbose = false;
    opts.jit = true;
    opts.compiler = 'shell';
    opts.jit_options.compiler = 'ccache gcc';
    opts.jit_options.flags = {'-O1'};
    opts.jit_temp_suffix = false;
    opts.qpsol = 'nlpsol';
    % opts.max_iter = 1;
    opts.qpsol_options.nlpsol = 'ipopt';
    opts.qpsol_options.nlpsol_options.ipopt.tol = 1e-7;
    opts.qpsol_options.nlpsol_options.ipopt.tiny_step_tol = 1e-20;
    opts.qpsol_options.nlpsol_options.ipopt.fixed_variable_treatment = 'make_constraint';
    opts.qpsol_options.nlpsol_options.ipopt.hessian_constant = 'yes';
    opts.qpsol_options.nlpsol_options.ipopt.jac_c_constant = 'yes';
    opts.qpsol_options.nlpsol_options.ipopt.jac_d_constant = 'yes';
    opts.qpsol_options.nlpsol_options.ipopt.accept_every_trial_step = 'yes';
    opts.qpsol_options.nlpsol_options.ipopt.mu_init = 1e-3;

    opts.qpsol_options.nlpsol_options.ipopt.print_level = 0;
    %opts.qpsol_options.nlpsol_options.print_time = false;
    opts.qpsol_options.nlpsol_options.ipopt.linear_solver = 'ma27';
    opts.qpsol_options.print_time = true;

    opti.solver('sqpmethod',opts);
end

%% BUILD PARAMETRIC PROBLEM
problemName = 'MPC';
% Build up input cell array:
inputCellArray = {x,u,T,Lp,np,xbeginp,xfinalp,ubeginp,measp,Ghatp,sigmap};
if withV
    inputCellArray{end+1} = uminp;
    inputCellArray{end+1} = umaxp;
end
if withA
    inputCellArray{end+1} = aminp;
    inputCellArray{end+1} = amaxp;
end
if withJ
    inputCellArray{end+1} = jminp;
    inputCellArray{end+1} = jmaxp;
end
if withOm
    inputCellArray{end+1} = omminp;
    inputCellArray{end+1} = ommaxp;
end
if withMaxDist
    inputCellArray{end+1} = maxDistp;
end

eval('problem = opti.to_function(problemName,inputCellArray,{x,u,T});');
problem.save('problem.casadi');

if strcmp(solver_str,'ipopt')==1
    if with_end
        problemIpoptB = problem;
        save 'problemIpoptB.mat' 'problemIpoptB'
    else
        problemIpoptA = problem;
        save 'problemIpoptA.mat' 'problemIpoptA'
    end
else
    if with_end
        problemSqpB = problem;
        save 'problemSqpB.mat' 'problemSqpB'
    else
        problemSqpA = problem;
        save 'problemSqpA.mat' 'problemSqpA'
    end
end

if log.exportBool == true
    if strcmp(solver_str,'ipopt')==1
        problem.save('problemIpopt.casadi');
    else
        problem.save('problemSqp.casadi');
    end
end
    
end