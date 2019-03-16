function sit = optim(sit,veh,count,solver)
% Formulate and solve the minimum time optimization problem.
% Vehicle model: diff-drive.

% parameters:
L       = veh.wheelBase;
n       = veh.Optim.n;
u_min   = veh.Optim.u_min;
u_max   = veh.Optim.u_max;
a_min   = veh.Optim.a_min;
a_max   = veh.Optim.a_max;
om_min  = veh.Optim.om_min;
om_max  = veh.Optim.om_max;
sigma_x = veh.Optim.sigma_x;
sigma_y = veh.Optim.sigma_y;
G_hat   = veh.Optim.G_hat;
min_scale = 0.2;

% get measurements:
meas    = sit.meas_tilde{end};

% CHECK POSITION AND ADAPT G-HAT IF NEEDED !!!!! %
G = checkPosition(meas,[0;0],sigma_x,sigma_y);
update_cycles = 0;
while update_cycles<=10 && G>G_hat
    G_hat = 1.5*G_hat;
    update_cycles = update_cycles + 1;
end
if update_cycles>0
    if G>G_hat
        fprintf(2, 'Tried to update G_hat, but too many update cycles - probably error will occur \n');
    else
        fprintf(2, 'Too high starting G_hat!! \n');
        fprintf(2, 'Updating G_hat to %f \n',G_hat);
    end
end

% ode:
ode = @(x,u)[0.5*(u(1)+u(2))*sin(x(3)); 0.5*(u(1)+u(2))*cos(x(3)); (1/L)*(u(2)-u(1))];

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

% initialize variables:
opti = casadi.Opti();

Lp = opti.parameter(1,1);
np = opti.parameter(1,1);
uminp = opti.parameter(1,1);
umaxp = opti.parameter(1,1);
aminp = opti.parameter(1,1);
amaxp = opti.parameter(1,1);
omminp = opti.parameter(1,1);
ommaxp = opti.parameter(1,1);
sigxp = opti.parameter(1,1);
sigyp = opti.parameter(1,1);
Ghatp = opti.parameter(1,1);
minscalep = opti.parameter(1,1);

opti.set_value(Lp, L);
opti.set_value(np, n);
opti.set_value(uminp, u_min);
opti.set_value(umaxp, u_max);
opti.set_value(aminp, a_min);
opti.set_value(amaxp, a_max);
opti.set_value(omminp, om_min);
opti.set_value(ommaxp, om_max);
opti.set_value(sigxp, sigma_x);
opti.set_value(sigyp, sigma_y);
opti.set_value(Ghatp, G_hat);
opti.set_value(minscalep, min_scale);

% initial and final positions + initial guess for time and states:
% HAS TO HAPPEN OUTSIDE OF FUNCTION:
x_begin = [0;0;0]; % always zero because problem solved in robot frame.
x_final = sit.localGoals{end};
T_init = norm(x_begin(1:2)-x_final(1:2))/u_max;
% x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
%     zeros(1,n+1)];
% x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
%     linspace(0,x_final(3),n+1)];
phi = atan2(x_final(3),n);
alpha = 0.5;
n_vec = linspace(0,n,n+1);
theta_star = alpha*sin(2*pi*n_vec/n);
theta_init = n_vec*sin(phi)+theta_star*cos(phi);
x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
    theta_init];

x    = opti.variable(3,n+1);
u    = opti.variable(2,n);
T    = opti.variable(1,n);

% if size(sit.Sol.T,2)==0
%     T_init  = sit.Init.T{end};
% else
%     %T_init  = sit.Sol.T{end};
%     T_init  = sit.Init.T{end};
% end

% constraints:
% TODO: add constraint for platform global acceleration
opti.subject_to(F(x(:,1:end-1),u,T/np)==x(:,2:end));
opti.subject_to(x(:,1)==x_begin);
opti.subject_to(x(:,end)==x_final);
opti.subject_to(uminp <= u(1,:) <= umaxp);
opti.subject_to(uminp <= u(2,:) <= umaxp);
opti.subject_to(u(1,:)+u(2,:) >= 0);
% opti.subject_to(a_min <= (n./T(2:end)).*diff(u(1,:)) <= a_max);
% opti.subject_to(a_min <= (n./T(2:end)).*diff(u(2,:)) <= a_max);
opti.subject_to(aminp <= np./T(2:end).*u(1,2:end)-np./T(1:end-1).*u(1,1:end-1) <= amaxp);
opti.subject_to(aminp <= np./T(2:end).*u(2,2:end)-np./T(1:end-1).*u(2,1:end-1) <= amaxp);
opti.subject_to(omminp <= (1/Lp)*(u(2,:)-u(1,:)) <= ommaxp);
opti.subject_to(T >= 0);
opti.subject_to(T(2:end)==T(1:end-1));
opti.subject_to(-minscalep<=diff(x(1,:))<=minscalep);
opti.subject_to(-minscalep<=diff(x(2,:))<=minscalep);

pos = casadi.SX.sym('pos',2);

Sum = 0;
for k=1:size(meas,1)
    th  = meas(k,1);
    r   = meas(k,2);
    p   = [-r*sin(th);r*cos(th)];
    g   = gaussianValue(pos,p,sigxp,sigyp);
    Sum = Sum + g;
end

costf = casadi.Function('costf',{pos},{Sum});

opti.subject_to(costf(x(1:2,:))<=Ghatp);

% initial guess:
% opti.set_initial(T, T_init);
% opti.set_initial(x, x_init);

% objective:
opti.minimize(sum(T)/np);

% solver:

Lp = opti.parameter(1,1);
np = opti.parameter(1,1);
uminp = opti.parameter(1,1);
umaxp = opti.parameter(1,1);
aminp = opti.parameter(1,1);
amaxp = opti.parameter(1,1);
omminp = opti.parameter(1,1);
ommaxp = opti.parameter(1,1);
sigxp = opti.parameter(1,1);
sigyp = opti.parameter(1,1);
Ghatp = opti.parameter(1,1);
minscalep = opti.parameter(1,1);

SQP = opti.to_function('SQP',{x,u,T,Lp,np,uminp,umaxp,aminp,amaxp,omminp,ommaxp,sigxp,sigyp,Ghatp,minscalep},{x,u,T});

if solver=='ipopt'
    if count==1
        opti.set_initial(T, T_init);
        opti.set_initial(x, x_init);
    else
        T0 = sit.Sol.T{end}(end);
        X0 = sit.Sol.X{end};
        U0 = sit.Sol.U{end};
        opti.set_initial(T,T0);
        opti.set_initial(x,X0);
        opti.set_initial(u,U0);
    end
    opti.solver('ipopt');
    
elseif solver=='qrqp'
    if count==1
        opti.solver('ipopt');
        sol = opti.solve();
        T0 = sol.value(T);
        X0 = sol.value(x);
        U0 = sol.value(u);
    else
        T0 = sit.Sol.T{end}(end);
        X0 = sit.Sol.X{end};
        U0 = sit.Sol.U{end};
        lam0 = sit.Sol.lamg{end};
    end

    opts = struct;
    opts.convexify_strategy = 'eigen-reflect';
    opts.verbose = true;
    opts.qpsol = 'qrqp';
    % opts.qpsol = 'qpoases';
    opts.qpsol_options.print_lincomb = 1;
    opts.dump_in = true;
    opts.dump = true;

    opti.solver('sqpmethod',opts);

    opti.set_initial(T,T0);
    opti.set_initial(x,X0);
    opti.set_initial(u,U0);
    if count>1
        opti.set_initial(opti.lam_g,lam0);
    end

    % opts.qpsol = 'qpoases';
    % opts.qpsol_options.sparse = true;
    % opts.qpsol_options.schur = true;
    % opti.solver('sqpmethod',opts);
end

% solve:
sol = opti.solve();

% extract solution:
T = sol.value(T);
X = sol.value(x);
U = sol.value(u);
lamg = sol.value(opti.lam_g);

% append to struct:
sit.Sol.X{end+1} = X;
sit.Sol.U{end+1} = U;
sit.Sol.T{end+1} = T;
sit.Sol.lamg{end+1} = lamg;

end