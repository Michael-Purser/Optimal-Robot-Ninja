function localPlanner = optim_temp(localPlanner,veh,solver_str,MPC_iteration)
% Temporary function acting as both problem builder and solver, while
% optim_setup is being debugged.
% As long as this function is used, parametric MPC is not implemented.

opti = casadi.Opti();

n           = localPlanner.params.horizon;
L           = veh.geometry.wheelBase;

withMaxDist = localPlanner.withMaxDistConstraints;
withV       = localPlanner.withVelocityConstraints;
withVPos    = localPlanner.withPositiveVelocityConstraints;
withA       = localPlanner.withAccelerationConstraints;
withJ       = localPlanner.withJerkConstraints;
withOm      = localPlanner.withOmegaConstraints;

u_min           = localPlanner.params.dynLimits.vel(1);
u_max           = localPlanner.params.dynLimits.vel(2);
a_min           = localPlanner.params.dynLimits.acc(1);
a_max           = localPlanner.params.dynLimits.acc(2);
j_min           = localPlanner.params.dynLimits.jerk(1);
j_max           = localPlanner.params.dynLimits.jerk(2);
om_min          = localPlanner.params.dynLimits.om(1);
om_max          = localPlanner.params.dynLimits.om(2);
beta            = localPlanner.params.maxDistBeta;
xglobal_x       = localPlanner.params.xglobalx';
xglobal_y       = localPlanner.params.xglobaly';
radii           = localPlanner.params.radii;
x_begin         = localPlanner.params.start;
x_final         = localPlanner.params.goal;
u_begin         = localPlanner.params.startVelocity;

warmStart       = localPlanner.warmStart;
withLinearEnd   = localPlanner.withLinearEndInitial;
linearEndSwitchDistance = localPlanner.linearEndSwitchDistance;

% ode:
ode  = @(x,u)[0.5*(u(1)+u(2))*sin(x(3)); 0.5*(u(1)+u(2))*cos(x(3)); (2/L)*(u(2)-u(1))];

% parametric global plan
xgx=casadi.interpolant('r','bspline',{linspace(0,1,size(xglobal_x,2))},xglobal_x,...
    struct('algorithm','smooth_linear','smooth_linear_frac',0.2));
xgy=casadi.interpolant('r','bspline',{linspace(0,1,size(xglobal_y,2))},xglobal_y,...
    struct('algorithm','smooth_linear','smooth_linear_frac',0.2));

% parametric obstacles
r=casadi.interpolant('r','bspline',{linspace(0,1,size(radii,2))},radii,...
    struct('algorithm','smooth_linear','smooth_linear_frac',0.2));

% make the max dist value:
maxDist = beta*norm(x_begin(1:2)-x_final(1:2))/n;

% initial values; distinction with or without linear end inital guess
if warmStart
    if withLinearEnd
        linearSelectorBool = (MPC_iteration==1 || norm(x_final(1:2))<linearEndSwitchDistance);
    else
        linearSelectorBool = (MPC_iteration==1);
    end
else
    linearSelectorBool = true; 
end
if strcmp(solver_str,'ipopt')==1
    fprintf('\t \t Using IPOPT solver \n');
    % if first iteration, make initial guesses; else 'warm-start' the
    % solver with previous solution:
    if linearSelectorBool 
        fprintf('\t \t NOT warm-started \n');
        theta_init = linspace(x_begin(3),x_final(3),n+1);
        x_init  = [linspace(x_begin(1),x_final(1),n+1);linspace(x_begin(2),x_final(2),n+1); ...
            theta_init];
        u_init  = zeros(2,n);
        T_init = norm(x_begin(1:2)-x_final(1:2))/u_max;
    else
        fprintf('\t \t Warm-started \n');
        x_init = localPlanner.sol.x;
        u_init = localPlanner.sol.u;
        T_init = localPlanner.sol.T;
    end
end

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
s    = opti.variable(1,n+1);
u    = opti.variable(2,n);
T    = opti.variable(1);

% multiple-shooting constraint
opti.subject_to(F(x(:,1:end-1),u,T./n)==x(:,2:end));

% state constraints
opti.subject_to(x(:,1)==x_begin);
opti.subject_to(x(:,end)==x_final);
if withMaxDist
    opti.subject_to(-maxDist<=diff(x(1,:))<=maxDist);
    opti.subject_to(-maxDist<=diff(x(2,:))<=maxDist);
end

% velocity constraints
if withV
    opti.subject_to(u_min <= u(1,:) <= u_max);
    opti.subject_to(u_min <= u(2,:) <= u_max);
    opti.subject_to(u(:,1) == u_begin);
end
if withVPos
    opti.subject_to(u(1,:)+u(2,:) >= 0);
end

% omega constraint
if withOm
    opti.subject_to(om_min <= (1/L)*(u(2,:)-u(1,:)) <= om_max);
end

% acceleration constraints
if withA
    opti.subject_to(a_min <= n/T*u(1,2:end)-n/T*u(1,1:end-1) <= a_max);
    opti.subject_to(a_min <= n/T*u(2,2:end)-n/T*u(2,1:end-1) <= a_max);
end

% jerk constraints
if withJ
    opti.subject_to(j_min <= ((n/T)^2)*u(1,3:end)-2*((n/T)^2)*u(1,2:end-1)+...
        ((n/T)^2)*u(1,1:end-2) <= j_max);
    opti.subject_to(j_min <= ((n/T)^2)*u(2,3:end)-2*((n/T)^2)*u(2,2:end-1)+...
        ((n/T)^2)*u(2,1:end-2) <= j_max);
end

% time constraints
opti.subject_to(T >= 0);

% obstacle avoidance 'tunnel' constraint
opti.subject_to(((x(1,:)-xgx(s(:))').^2 + (x(2,:)-xgy(s(:))').^2)<=r(s(:)').^2);

% constraints on parameter s
opti.subject_to(s(1) == 0);
opti.subject_to(s(n+1) == 1);
opti.subject_to(0<=s(:)<=1);
opti.subject_to(diff(s(:))<=1/n);

% initial guesses
opti.set_initial(s,linspace(0,1,n+1));
opti.set_initial(x,x_init);
opti.set_initial(u,u_init);
opti.set_initial(T,T_init);

% objective:
opti.minimize(T);

% solver:
options.ipopt.print_level = 5;
opti.solver('ipopt',options);

% solve:
sol = opti.solve();

% append to struct:
localPlanner.sol.x = sol.value(x);
localPlanner.sol.u = sol.value(u);
localPlanner.sol.T = sol.value(T);
localPlanner.sol.s = sol.value(s);
localPlanner.sol.stats = opti.stats();
localPlanner.sol.success = true;
