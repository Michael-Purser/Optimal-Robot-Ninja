% SIMPLE TEST OF SQP SOLVER:
% With holonomic robot

clear;
close all;
clc;

% get problem parameters:
n       = 200;
u_min   = -1;
u_max   = 1;
v_min   = -2;
v_max   = 2;

% Define the ode:
ode = @(x,u)[x(3); x(4); u(1); u(2)];

% states integration:
h = casadi.SX.sym('h');

x = casadi.SX.sym('x',4);
u = casadi.SX.sym('u',2);
k1 = ode(x,       u); 
k2 = ode(x+h/2*k1,u);
k3 = ode(x+h/2*k2,u);
k4 = ode(x+h*k3,  u);

xf = x+ h/6 * (k1 + 2*k2 + 2*k3 + k4);
F = casadi.Function('F',{x,u,h},{xf});

% initialize opti:
opti = casadi.Opti();
T    = opti.variable(1,1);
x    = opti.variable(4,n+1);
u    = opti.variable(2,n);

% Initial and final positions + initial guess for time and states:
x_begin = [0;0;0;0]; % Always at zero because problem solved in robot frame.
x_final = [10;10;0;0];

% Solve optimization problem:

opti.subject_to(F(x(:,1:end-1),u,T/n)==x(:,2:end));
opti.subject_to(x(:,1)==x_begin);
opti.subject_to(x(:,end)==x_final);
opti.subject_to(v_min <= x(3,:) <= v_max);
opti.subject_to(v_min <= x(4,:) <= v_max);
opti.subject_to(u_min <= u(1,:) <= u_max);
opti.subject_to(u_min <= u(2,:) <= u_max);
opti.subject_to(T >= 0);
% min_scale = 0.1;
% opti.subject_to(-min_scale<=diff(x(1,:))<=min_scale);
% opti.subject_to(-min_scale<=diff(x(2,:))<=min_scale);

pos = casadi.SX.sym('pos',2);


T_init  = norm(x_begin(1:2)-x_final(1:2))/v_max;
x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
    zeros(1,n+1); zeros(1,n+1)];

% Set initial guess:
opti.set_initial(T, T_init);
opti.set_initial(x, x_init);

% Objective function: 
opti.minimize(T);

% options = struct;
% options.expand = true;
% opti.solver('ipopt',options);

opts.qpsol = 'qpoases';
opts.qpsol_options.sparse = true;
opts.qpsol_options.schur = true;
opti.solver('sqpmethod',opts);

% Solve optimization problem:
sol = opti.solve();

% Extract solution:
T = sol.value(T);
X = sol.value(x);
U = sol.value(u);

figure;
plot(X(1,:),X(2,:),'.');

