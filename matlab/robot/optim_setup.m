function problem = optim_setup(MPC,veh,solver_str)

% initialize variables:
opti = casadi.Opti();

max_meas    = 1000;
n           = MPC.nav.opt.horizon;
L           = veh.geometry.wheelBase;

Lp          = opti.parameter(1,1);
np          = opti.parameter(1,1);
uminp       = opti.parameter(1,1);
umaxp       = opti.parameter(1,1);
aminp       = opti.parameter(1,1);
amaxp       = opti.parameter(1,1);
omminp      = opti.parameter(1,1);
ommaxp      = opti.parameter(1,1);
sigxp       = opti.parameter(1,1);
sigyp       = opti.parameter(1,1);
Ghatp       = opti.parameter(1,1);
minscalep   = opti.parameter(1,1);
measp       = opti.parameter(max_meas,2);
xbeginp     = opti.parameter(3,1);
xfinalp     = opti.parameter(3,1);

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

x    = opti.variable(3,n+1);
u    = opti.variable(2,n);
T    = opti.variable(1,n);

% [dusqmax,samax] = getULimits(veh,0.8);

% constraints:
% TODO: add constraint for platform global acceleration
% 	    make actuator acceleration limits dependent on current velocity
% 	    add jerk contraints to avoid inf accelerations
opti.subject_to(F(x(:,1:end-1),u,T./np)==x(:,2:end));
opti.subject_to(x(:,1)==xbeginp);
opti.subject_to(x(:,end)==xfinalp);
opti.subject_to(uminp <= u(1,:) <= umaxp);
opti.subject_to(uminp <= u(2,:) <= umaxp);
opti.subject_to(u(1,:)+u(2,:) >= 0);
% opti.subject_to(abs(u(1,:).^2-u(2,:).^2) <= dusqmax);
% opti.subject_to(a_min <= (n./T(2:end)).*diff(u(1,:)) <= a_max);
% opti.subject_to(a_min <= (n./T(2:end)).*diff(u(2,:)) <= a_max);
opti.subject_to(aminp <= np./T(2:end).*u(1,2:end)-np./T(1:end-1).*u(1,1:end-1) <= amaxp);
opti.subject_to(aminp <= np./T(2:end).*u(2,2:end)-np./T(1:end-1).*u(2,1:end-1) <= amaxp);
opti.subject_to(omminp <= (1/Lp)*(u(2,:)-u(1,:)) <= ommaxp);
opti.subject_to(T >= 0);
opti.subject_to(T(2:end)==T(1:end-1));
opti.subject_to(-minscalep<=diff(x(1,:))<=minscalep);
opti.subject_to(-minscalep<=diff(x(2,:))<=minscalep);

pos = casadi.MX.sym('pos',2);

th  = measp(:,1);
r   = measp(:,2);
p   = [-r.*sin(th) r.*cos(th)];
g   = gaussianValue(p,pos,sigxp,sigyp);
costf = casadi.Function('costf',{pos},{sum(g)});

opti.subject_to(costf(x(1:2,:))<=Ghatp);

% objective:
opti.minimize(sum(T)/np);

% solver:
opts = struct;
opts.convexify_strategy = 'eigen-reflect';
opts.verbose = true;
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

% opti.solver('sqpmethod',opts);
if strcmp(solver_str,'ipopt')==1
    opti.solver('ipopt');
else
    opti.solver('sqpmethod',opts);
end

problem = opti.to_function('MPC',{x,u,T,Lp,np,uminp,umaxp,aminp,amaxp,omminp,ommaxp,Ghatp,minscalep,xbeginp,xfinalp,measp,sigxp,sigyp},{x,u,T});
problem.save('problem.casadi');

if strcmp(solver_str,'ipopt')==1
    problemIpopt = problem;
    save 'problemIpopt.mat' 'problemIpopt'
else
    problemSqp = problem;
    save 'problemSqp.mat' 'problemSqp'
end

if MPC.log.exportBool == true
    if strcmp(solver_str,'ipopt')==1
        problem.save('problemIpopt.casadi');
    else
        problem.save('problemSqp.casadi');
    end
end
    
end