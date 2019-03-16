% Test of a simple time optimization of a acceleration-steered diff-drive
% robot with no obstacles and multiple-shooting using Runge-Kutta 
% integration.

clear;
close all;

addpath('~/Downloads/casadi-matlabR2014a-v3.4.5/');
opti = casadi.Opti();

n = 500;

T = opti.variable(1,1);
x = opti.variable(3,n+1);
u = opti.variable(2,n+1);

% Initial and final positions + initial guess for time and states:
x_begin = [0;0;pi/2];
x_final = [3;3;pi/2];
T_init  = 5;
x_init  = [linspace(0,x_final(1),n+1);linspace(0,x_final(2),n+1); ...
    zeros(1,n+1)];

% Constraint values:
u_min = 0;
u_max = 1;
v_min = -8;
v_max = 8;
a_min = -2;
a_max = 2;

% Define the ode:
L = 0.2;
ode = @(x,u)[0.5*(u(1)+u(2))*cos(x(3)); 0.5*(u(1)+u(2))*sin(x(3)); (1/L)*(u(2)-u(1))];

% States integration:
h  = T/n;
for k=1:(n)
    k1 = ode(x(:,k),       u(:,k)); 
    k2 = ode(x(:,k)+h/2*k1,u(:,k));
    k3 = ode(x(:,k)+h/2*k2,u(:,k));
    k4 = ode(x(:,k)+h*k3,  u(:,k));
    opti.subject_to(x(:,k+1) == x(:,k) + h/6 * (k1 + 2*k2 + 2*k3 + k4));
end

% Solve optimization problem:

% Path constraints:
opti.subject_to(x(:,1)==x_begin);
opti.subject_to(x(:,end)==x_final);
% opti.subject_to(v_min <= x(3,:) <= v_max);
% opti.subject_to(v_min <= x(4,:) <= v_max);
opti.subject_to(u_min <= u(1,:) <= u_max);
opti.subject_to(u_min <= u(2,:) <= u_max);
opti.subject_to(T >= 0);

% Set initial guess:
opti.set_initial(T, T_init);
opti.set_initial(x, x_init);

% Objective function:
opti.minimize(T);
opti.solver('ipopt');

% Solve optimization problem:
sol = opti.solve();

% Extract solution and plot:
T = sol.value(T);
X = sol.value(x);
U = sol.value(u);

t = linspace(0,T,n);

figure;
plot(X(1,:),X(2,:),'o-');
axis equal;
title('Optimal trajectory');

% figure;
% subplot(2,2,1);
% plot(t,X(3,:)); title('v_{x}(t)');
% subplot(2,2,3);
% plot(t,X(4,:)); title('v_{y}(t)');
% subplot(2,2,2);
% plot(t,U(1,:)); title('a_{x}(t)');
% subplot(2,2,4);
% plot(t,U(2,:)); title('a_{y}(t)');