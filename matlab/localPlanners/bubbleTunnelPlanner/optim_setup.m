function problem = optim_setup(localPlanner,veh,log,solver_str,with_end)

% initialize variables:
opti = casadi.Opti();

max_meas    = 1000;
max_gplan   = 1000;

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
maxDistp    = opti.parameter(1,1);
rp          = opti.parameter(max_meas,2);
xgxp        = opti.parameter(max_gplan,1);
xgyp        = opti.parameter(max_gplan,1);
xbeginp     = opti.parameter(3,1);
xfinalp     = opti.parameter(3,1);
ubeginp     = opti.parameter(2,1);

% ode:
ode  = @(x,u)[0.5*(u(1)+u(2))*sin(x(3)); 0.5*(u(1)+u(2))*cos(x(3)); (2/L)*(u(2)-u(1))];
% 
% % parametric global plan
% xgx=casadi.interpolant('r','bspline',{linspace(0,1,size(xglobalxp,1))},xglobalxp',...
%     struct('algorithm','smooth_linear','smooth_linear_frac',0.2));
% xgy=casadi.interpolant('r','bspline',{linspace(0,1,size(xglobalyp,1))},xglobalyp',...
%     struct('algorithm','smooth_linear','smooth_linear_frac',0.2));

% parametric obstacles
% r=casadi.interpolant('r','bspline',{linspace(0,1,size(radiip,2))},radiip,...
%     struct('algorithm','smooth_linear','smooth_linear_frac',0.2));

% initial values
theta_init = linspace(xbeginp(3),xfinalp(3),n+1);
x_init  = [linspace(xbeginp(1),xfinalp(1),n+1)';linspace(xbeginp(2),xfinalp(2),n+1)'; ...
    theta_init'];
u_init  = zeros(2,n);
T_init = norm(xbeginp(1:2)-xfinalp(1:2))/umaxp;

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

% omega constraint
if withOm
    opti.subject_to(omminp <= (1/L)*(u(2,:)-u(1,:)) <= ommaxp);
end

% acceleration constraints
if withA
    opti.subject_to(aminp <= n/T*u(1,2:end)-n/T*u(1,1:end-1) <= amaxp);
    opti.subject_to(aminp <= n/T*u(2,2:end)-n/T*u(2,1:end-1) <= amaxp);
end

% jerk constraints
if withJ
    opti.subject_to(jminp <= ((n/T)^2)*u(1,3:end)-2*((n/T)^2)*u(1,2:end-1)+...
        ((n/T)^2)*u(1,1:end-2) <= jmaxp);
    opti.subject_to(jminp <= ((n/T)^2)*u(2,3:end)-2*((n/T)^2)*u(2,2:end-1)+...
        ((n/T)^2)*u(2,1:end-2) <= jmaxp);
end

% time constraints
opti.subject_to(T >= 0);

% obstacle avoidance 'tunnel' constraint
opti.subject_to(((x(1,:)-xgxp(s(:))').^2 + (x(2,:)-xgyp(s(:))').^2)<=rp(s(:)').^2);

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
options.ipopt.print_level = 0;
opti.solver('ipopt',options);

%% BUILD PARAMETRIC PROBLEM
problemName = 'MPC';
% Build up input cell array:
inputCellArray = {x,u,T,Lp,np,xbeginp,xfinalp,ubeginp,xglobalxp,xglobalyp,radiip};
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

if with_end
    problemIpoptB = problem;
    save 'problemIpoptB.mat' 'problemIpoptB'
else
    problemIpoptA = problem;
    save 'problemIpoptA.mat' 'problemIpoptA'
end

if log.exportBool == true
    if strcmp(solver_str,'ipopt')==1
        problem.save('problemIpopt.casadi');
    end
end


