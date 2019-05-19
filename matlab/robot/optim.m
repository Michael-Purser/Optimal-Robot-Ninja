function MPC = optim(MPC,veh)
% Formulate and solve the minimum time optimization problem.
% Vehicle model: diff-drive.

% parameters:
L           = veh.geometry.wheelBase;
n           = MPC.nav.opt.horizon;
u_min       = MPC.nav.opt.dynamicLimits.vel(1);
u_max       = MPC.nav.opt.dynamicLimits.vel(2);
a_min       = MPC.nav.opt.dynamicLimits.acc(1);
a_max       = MPC.nav.opt.dynamicLimits.acc(2);
om_min      = MPC.nav.opt.dynamicLimits.om(1);
om_max      = MPC.nav.opt.dynamicLimits.om(2);
sigma       = MPC.nav.opt.sigma;
Ghat        = MPC.nav.opt.Ghat;
maxDist     = MPC.nav.opt.maxDist;
solver      = MPC.nav.opt.solver;
max_meas    = 1000;

% get measurements (in cartesian coordinates):
meas        = MPC.nav.opt.obst;
meas        = [meas;20*ones(max_meas-size(meas,1),2)];

% check position; adapt G_hat if needed:
G = checkPosition(meas,[0 0],sigma,sigma);
update_cycles = 0;
while update_cycles<=10 && G>Ghat
    Ghat = 1.5*Ghat;
    update_cycles = update_cycles + 1;
end
if update_cycles>0
    if G>Ghat
        fprintf(2, 'Tried to update G_hat, but too many update cycles - probably error will occur \n');
    else
        fprintf(2, 'Too high starting G_hat!! \n');
        fprintf(2, 'Updating G_hat to %f \n',Ghat);
    end
end

% initialize opti:
opti = casadi.Opti();

% initial and final positions + initial guess for time and states:
x_begin = MPC.nav.opt.start;
x_final = MPC.nav.opt.goal;
u_begin = MPC.nav.currentVelocity;

if strcmp(solver,'ipopt')==1
    fprintf('Using IPOPT solver \n');
    % if first iteration, make initial guesses; else 'warm-start' the
    % solver with previous solution:
    if MPC.nav.k==1
%         phi         = atan2(x_final(3),n);
%         alpha       = 0.5;
%         n_vec       = linspace(0,n,n+1);
%         theta_star  = alpha*sin(2*pi*n_vec/n);
%         theta_init  = n_vec*sin(phi)+theta_star*cos(phi);
        theta_init = linspace(0,x_final(3),n+1);
        x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
            theta_init];
        u_init  = zeros(2,n);
        T_init = norm(x_begin(1:2)-x_final(1:2))/u_max;
    else
        x_init = MPC.nav.opt.sol.x;
        u_init = MPC.nav.opt.sol.u;
        T_init = MPC.nav.opt.sol.T;
    end
    % select problem to solve depending on wether endgoal is in view or
    % not:
    if MPC.nav.goalInView
        fprintf('\t Goal in view \n');
        problem    = MPC.nav.problemIpoptB;
    else
        fprintf('\t Goal not in view \n');
        problem    = MPC.nav.problemIpoptA;
    end
    
else
    fprintf('Using SQP solver - first iterations are IPOPT \n');
    if MPC.nav.k<3
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
        problem = MPC.nav.problemIpoptA;
    else
        x_init  = MPC.nav.opt.sol.x;
        u_init  = MPC.nav.opt.sol.u;
        T_init  = MPC.nav.opt.sol.T;
        % select problem to solve depending on wether endgoal is in view or
        % not:
        if MPC.nav.goalInView
            fprintf('\t Goal in view \n');
            problem    = MPC.nav.problemSqpB;
        else
            fprintf('\t Goal not in view \n');
            problem    = MPC.nav.problemSqpA;
        end
    end
end

% log the initial guesses
MPC.nav.opt.init.x = x_init;
MPC.nav.opt.init.u = u_init;
MPC.nav.opt.init.T = T_init;

[X,U,T] = problem(x_init,u_init,T_init,L,n,u_min,u_max,a_min,a_max,om_min,...
        om_max,Ghat,maxDist,x_begin,x_final,u_begin,meas,sigma);

% append to struct:
MPC.nav.opt.sol.x = opti.value(X);
MPC.nav.opt.sol.u = opti.value(U);
MPC.nav.opt.sol.T = opti.value(T);

% lamg = sol.value(opti.lam_g);

% append to struct:
% sit.Sol.lamg{end+1} = lamg;

end
