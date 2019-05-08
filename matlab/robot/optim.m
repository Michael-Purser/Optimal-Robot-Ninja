function sit = optim(sit,veh,count,solver)
% Formulate and solve the minimum time optimization problem.
% Vehicle model: diff-drive.

% parameters:
L           = veh.wheelBase;
n           = veh.Optim.n;
u_min       = veh.Optim.u_min;
u_max       = veh.Optim.u_max;
a_min       = veh.Optim.a_min;
a_max       = veh.Optim.a_max;
om_min      = veh.Optim.om_min;
om_max      = veh.Optim.om_max;
sigma_x     = veh.Optim.sigma_x;
sigma_y     = veh.Optim.sigma_y;
G_hat       = veh.Optim.G_hat;
min_scale   = 0.2;
max_meas    = 1000;

% get measurements:
meas    = sit.meas_tilde{end};
meas = [meas;20*ones(max_meas-size(meas,1),2)];

% check position; adapt G_hat if needed:
G = checkPosition(meas,[0 0],sigma_x,sigma_y);
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

% initialize variables:
opti = casadi.Opti();

% initial and final positions + initial guess for time and states:
x_begin = [0;0;0];  % always zero because problem solved in robot frame.
x_final = sit.localGoals{end};



if strcmp(solver,'ipopt')==1
    if count==1
        % initial guesses for first iteration
        phi         = atan2(x_final(3),n);
        alpha       = 0.5;
        n_vec       = linspace(0,n,n+1);
        theta_star  = alpha*sin(2*pi*n_vec/n);
        theta_init  = n_vec*sin(phi)+theta_star*cos(phi);
        x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
            theta_init];
        u_init  = zeros(2,n);
        T_init = norm(x_begin(1:2)-x_final(1:2))/u_max;
    else
        x_init = sit.Sol.X{end};
        u_init = sit.Sol.U{end};
        T_init = sit.Sol.T{end}(end);
    end
    MPC     = sit.ipoptSolver;
    
else
    if count<5
        % initial guesses for first iteration
        phi         = atan2(x_final(3),n);
        alpha       = 0.5;
        n_vec       = linspace(0,n,n+1);
        theta_star  = alpha*sin(2*pi*n_vec/n);
        theta_init  = n_vec*sin(phi)+theta_star*cos(phi);
        x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
            theta_init];
        u_init  = zeros(2,n);
        T_init  = norm(x_begin(1:2)-x_final(1:2))/u_max;
        MPC     = sit.ipoptSolver;
    else
        x_init  = sit.Sol.X{end};
        u_init  = sit.Sol.U{end};
        T_init  = sit.Sol.T{end}(end);
        MPC     = sit.sqpSolver;
    end
end

[X,U,T] = MPC(x_init,u_init,T_init,L,n,u_min,u_max,a_min,a_max,om_min,...
        om_max,G_hat,min_scale,x_begin,x_final,meas,sigma_x,sigma_y);

% append to struct:
sit.Sol.X{end+1} = opti.value(X);
sit.Sol.U{end+1} = opti.value(U);
sit.Sol.T{end+1} = opti.value(T);

% lamg = sol.value(opti.lam_g);

% append to struct:
% sit.Sol.lamg{end+1} = lamg;

end
