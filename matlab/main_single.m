% BEFORE RUNNING:
% execute 'makeEnv()', 'makeVeh()' and 'makeSit()' in command line (in that
% order); they can be found in the 'simulation' folder.

clear;
close all;
clc;

addpath('~/Downloads/casadi/install/matlab/');
addpath('./data/');
addpath('./rastar/');
addpath('./robot/');
addpath('./simulation/');
addpath('./visualisation/');

% situation:
sitStr = '1_8_1';


%% SIMULATION

% load situation, environment and vehicle:
eval(['load ./data/sit',sitStr,'.mat;']);
eval(['load ./data/veh',num2str(sit.vehNum),'.mat;']);
eval(['load ./data/env',num2str(sit.envNum),'.mat;']);

% manual overrides:
veh.Sensor.horizon  = 5;
veh.Sensor.noiseamp = 0;
sit.states          = {sit.startState};
sit.goalState       = [10;10;0];
veh.Optim.n         = 200;

% simulate sensor:
env = relevantObst(sit,veh,env);
sit = sensor(sit,veh,env);

% make global path to follow:
pathManual = [sit.goalState(1:2)];
sit.globalVisited = [];
sit.globalNotVisited = pathManual;


%% VEHICLE

% % make vehicle map:
% sit = makeMap(sit,veh);
% sit = addMeasurementsToMap(sit,veh,2,1);
% 
% % make G-landscape using sensor measurements:
% sit = addGaussianToMap(sit,veh);

% express goal in vehicle frame:
sit = getLocalGoal(sit);

% % get geometric path to goal:
% sit = globalPlanner(sit,veh);


%% OPTIMISATION:

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

% get measurements:
meas    = sit.meas{end};

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



x    = opti.variable(3,n+1);
u    = opti.variable(2,n);
T    = opti.variable(1,n);

% initial and final positions + initial guess for time and states:
x_begin = [0;0;0]; % always zero because problem solved in robot frame.
x_final = sit.localGoals{end};
if size(sit.Sol.T,2)==0
    T_init  = sit.Init.T{end};
else
    %T_init  = sit.Sol.T{end};
    T_init  = sit.Init.T{end};
end
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

% constraints:
% TODO: add constraint for platform global acceleration
opti.subject_to(F(x(:,1:end-1),u,T/n)==x(:,2:end));
opti.subject_to(x(:,1)==x_begin);
opti.subject_to(x(:,end)==x_final);
opti.subject_to(u_min <= u(1,:) <= u_max);
opti.subject_to(u_min <= u(2,:) <= u_max);
opti.subject_to(u(1,:)+u(2,:) >= 0);
% opti.subject_to(a_min <= (n./T(2:end)).*diff(u(1,:)) <= a_max);
% opti.subject_to(a_min <= (n./T(2:end)).*diff(u(2,:)) <= a_max);
opti.subject_to(a_min <= n./T(2:end).*u(1,2:end)-n./T(1:end-1).*u(1,1:end-1) <= a_max);
opti.subject_to(a_min <= n./T(2:end).*u(2,2:end)-n./T(1:end-1).*u(2,1:end-1) <= a_max);
opti.subject_to(om_min <= (1/L)*(u(2,:)-u(1,:)) <= om_max);
opti.subject_to(T >= 0);
opti.subject_to(T(2:end)==T(1:end-1));
min_scale = 0.1;
opti.subject_to(-min_scale<=diff(x(1,:))<=min_scale);
opti.subject_to(-min_scale<=diff(x(2,:))<=min_scale);

pos = casadi.SX.sym('pos',2);

Sum = 0;
for k=1:size(meas,1)
    th  = meas(k,1);
    r   = meas(k,2);
    p   = [-r*sin(th);r*cos(th)];
    g   = gaussianValue(pos,p,sigma_x,sigma_y);
    Sum = Sum + g;
end

costf = casadi.Function('costf',{pos},{Sum});

opti.subject_to(costf(x(1:2,:))<=G_hat);

T_init = norm(x_begin(1:2)-x_final(1:2))/u_max;

% initial guess:
opti.set_initial(T, T_init);
opti.set_initial(x, x_init);

% objective:
opti.minimize(sum(T)/n);
opti.solver('ipopt');
sol = opti.solve();
T0 = sol.value(T);
X0 = sol.value(x);
U0 = sol.value(u);


opts = struct;
opts.convexify_strategy = 'eigen-reflect';
opts.verbose = true;
opts.qpsol = 'osqp';
% opts.qpsol = 'qpoases';
opts.qpsol_options.osqp.alpha = 1;
opts.qpsol_options.osqp.eps_abs = 1e-8;
opts.qpsol_options.osqp.eps_rel = 1e-8;
opts.qpsol_options.osqp.max_iter = 1e7;
opts.qpsol_options.dump = true;
opti.solver('sqpmethod',opts);

opti.set_initial(T,T0);
opti.set_initial(x,X0);
opti.set_initial(u,U0);

opts.qpsol = 'qpoases';
opts.qpsol_options.sparse = true;
opts.qpsol_options.schur = true;
opti.solver('sqpmethod',opts);

% solve:
sol = opti.solve();

% extract solution:
T = sol.value(T);
X = sol.value(x);
U = sol.value(u);

% append to struct:
sit.Sol.X{end+1} = X;
sit.Sol.U{end+1} = U;
sit.Sol.T{end+1} = T;


%% POST-PROCESSING 
% (other functions available; see folder 'visualization')

% check obstacle constraint satisfaction:
sit = checkSolution(sit,veh,1);

% plot optimal trajectory (in local and global frames):
plotLocalSol(sit,veh,1);
plotSol(sit,veh,env);

% make movie:
% close all;
% makeMov(sit,veh,env,'test6',3.0,3.0);