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
x_begin = [0;0;0]; % always zero because problem solved in robot frame.
x_final = sit.localGoals{end};

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

MPC = sit.solver;
[X,U,T] = MPC(x_init,u_init,T_init,L,n,u_min,u_max,a_min,a_max,om_min,...
    om_max,G_hat,min_scale,x_begin,x_final,meas,sigma_x,sigma_y);

% append to struct:
sit.Sol.X{end+1} = opti.value(X);
sit.Sol.U{end+1} = opti.value(U);
sit.Sol.T{end+1} = opti.value(T);





% [dusqmax,samax] = getULimits(veh,0.8);
% 
% % constraints:
% % TODO: add constraint for platform global acceleration
% % ------> make actuator acceleration limits dependent on current velocity
% % ------> add jerk contraints to avoid inf accelerations
% opti.subject_to(F(x(:,1:end-1),u,T/np)==x(:,2:end));
% opti.subject_to(x(:,1)==xbeginp);
% opti.subject_to(x(:,end)==xfinalp);
% opti.subject_to(uminp <= u(1,:) <= umaxp);
% opti.subject_to(uminp <= u(2,:) <= umaxp);
% opti.subject_to(u(1,:)+u(2,:) >= 0);
% % opti.subject_to(abs(u(1,:).^2-u(2,:).^2) <= dusqmax);
% % opti.subject_to(a_min <= (n./T(2:end)).*diff(u(1,:)) <= a_max);
% % opti.subject_to(a_min <= (n./T(2:end)).*diff(u(2,:)) <= a_max);
% opti.subject_to(aminp <= np./T(2:end).*u(1,2:end)-np./T(1:end-1).*u(1,1:end-1) <= amaxp);
% opti.subject_to(aminp <= np./T(2:end).*u(2,2:end)-np./T(1:end-1).*u(2,1:end-1) <= amaxp);
% opti.subject_to(omminp <= (1/Lp)*(u(2,:)-u(1,:)) <= ommaxp);
% opti.subject_to(T >= 0);
% opti.subject_to(T(2:end)==T(1:end-1));
% opti.subject_to(-minscalep<=diff(x(1,:))<=minscalep);
% opti.subject_to(-minscalep<=diff(x(2,:))<=minscalep);
% 
% pos = casadi.MX.sym('pos',2);
% 
% th  = measp(:,1);
% r   = measp(:,2);
% p   = [-r.*sin(th) r.*cos(th)];
% g   = gaussianValue(p,pos,sigxp,sigyp);
% costf = casadi.Function('costf',{pos},{sum(g)});
% 
% opti.subject_to(costf(x(1:2,:))<=Ghatp);
% 
% % objective:
% opti.minimize(sum(T)/np);
% 
% % solver:
% % TODO: add a flag to get each time a waypoint has changed, reuse above
% % defined T_init?? (as T0 may vary too much).
% if strcmp(solver,'ipopt')==1
%     if count==1
%         opti.set_initial(T, T_init);
%         opti.set_initial(x, x_init);
%         opti.set_initial(u, u_init);
%     else
%         T0 = sit.Sol.T{end}(end);
%         X0 = sit.Sol.X{end};
%         U0 = sit.Sol.U{end};
%         opti.set_initial(T,T0);
%         opti.set_initial(x,X0);
%         opti.set_initial(u,U0);
%     end
%     opts = struct;
%     opts.error_on_fail = true;
%     opti.solver('ipopt',opts);
% %     opti.solver('ipopt',struct('dump',true));
%     
% elseif strcmp(solver,'qrqp')==1
%     if count<3
%         opti.set_initial(T, T_init);
%         opti.set_initial(x, x_init);
%         opti.solver('ipopt');
%         %sol = opti.solve();
%         %T0 = sol.value(T);
%         %X0 = sol.value(x);
%         %U0 = sol.value(u); 
%         T0 = T_init;
%         X0 = x_init;
%         U0 = u_init;
% %         lam0 = sol.value(opti.lam_g);
%     else
%         T0 = sit.Sol.T{end}(end);
%         X0 = sit.Sol.X{end};
%         U0 = sit.Sol.U{end};
% %         lam0 = sit.Sol.lamg{end};
%     end
% 
%     opts = struct;
%     opts.convexify_strategy = 'eigen-reflect';
%     opts.verbose = true;
%     opts.jit = true;
%     opts.compiler = 'shell';
%     opts.jit_options.compiler = 'ccache gcc';
%     opts.jit_options.flags = {'-O1'};
%     opts.jit_temp_suffix = false;
%     opts.qpsol = 'nlpsol';
%     %opts.max_iter = 2; %%%
%     opts.qpsol_options.nlpsol = 'ipopt';
%     opts.qpsol_options.nlpsol_options.ipopt.tol = 1e-7;
%     opts.qpsol_options.nlpsol_options.ipopt.tiny_step_tol = 1e-20;
%     opts.qpsol_options.nlpsol_options.ipopt.fixed_variable_treatment = 'make_constraint';
%     opts.qpsol_options.nlpsol_options.ipopt.hessian_constant = 'yes';
%     opts.qpsol_options.nlpsol_options.ipopt.jac_c_constant = 'yes';
%     opts.qpsol_options.nlpsol_options.ipopt.jac_d_constant = 'yes';
%     opts.qpsol_options.nlpsol_options.ipopt.accept_every_trial_step = 'yes';
%     opts.qpsol_options.nlpsol_options.ipopt.mu_init = 1e-3;
% 
%     opts.qpsol_options.nlpsol_options.ipopt.print_level = 0;
%     %opts.qpsol_options.nlpsol_options.print_time = false;
%     opts.qpsol_options.nlpsol_options.ipopt.linear_solver = 'ma27';
%     opts.qpsol_options.print_time = true;
%  
% 
%     opti.solver('sqpmethod',opts);
% 
%     opti.set_initial(T,T0);
%     opti.set_initial(x,X0);
%     opti.set_initial(u,U0);
% %     if count>1
% %         opti.set_initial(opti.lam_g,lam0);
% %     end
% 
%     % opts.qpsol = 'qpoases';
%     % opts.qpsol_options.sparse = true;
%     % opts.qpsol_options.schur = true;
%     % opti.solver('sqpmethod',opts);
% end
% 
% % solve:
% % sol = opti.solve_limited();
% 
% % export the solver times if the boolean is true
% if sit.log_bool == 1
%     sit.log_vector(end+1) = sol.stats.t_wall_solver;
% end
% 
% % export the solver if it's bthe first iteration and the boolean is true
% if count==1 && sit.export_bool==1
%     MPC = opti.to_function('MPC',{x,u,T,Lp,np,uminp,umaxp,aminp,amaxp,omminp,ommaxp,Ghatp,minscalep,xbeginp,xfinalp,measp},{x,u,T});
% %     MPC = opti.to_function('MPC',{x,u,T,L,n,u_min,u_max,a_min,a_max,om_min,om_max,G_hat,min_scale,x_begin,x_final,meas},{x,u,T});
%     sit.solver = MPC;
%     %MPC.save('MPC.casadi');
% end
% 
% % extract solution:
% T = sol.value(T);
% X = sol.value(x);
% U = sol.value(u);
% lamg = sol.value(opti.lam_g);

% append to struct:
% sit.Sol.lamg{end+1} = lamg;

end
