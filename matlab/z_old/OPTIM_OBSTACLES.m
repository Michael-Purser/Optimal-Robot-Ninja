% Test of a simple time optimization of an acceleration-steered point mass
% with obstacles and multiple-shooting using Runge-Kutta integration.

clear;
close all;

addpath('~/Downloads/casadi-matlabR2014a-v3.4.5/'); % change to your casadi path
opti = casadi.Opti();

% Number of points used for multiple shooting:
n = 130;

T = opti.variable(1,1);
x = opti.variable(4,n);
u = opti.variable(2,n);

% Initial and final positions + initial guess for time and states:
% (state is [x;y;v_x;v_y]):
x_begin = [0;0;0;0];
x_final = [10;10;0;0];
T_init  = 0;
x_init  = [linspace(0,x_final(1),n);linspace(0,x_final(2),n); ...
    zeros(1,n); zeros(1,n)];

% Make simple cases of rangefinder measurement data, called 'meas'.
% Structure of 'meas':
%       [theta1  r1;
%       [theta2  r2;
%       ...]
% (Every row represents a measurement)

% meas = [-pi/4 3];
% meas = [pi/6 3; -pi/4 5; -pi/3 3];
meas = [-pi/6 3; -pi/4 5; -pi/3 3];  % this one gives problems, even
                                     % when 'n' is made larger.

% Gaussian parameters:
sigma_x = 0.5;
sigma_y = 0.5;

% Constraint values:
u_min = -2;
u_max = 2;
v_min = -10;
v_max = 10;
G_hat = 0.3;

% Define ODE:
ode = @(x,u)[x(3); x(4); u(1); u(2)];

% States integration:
h  = T/n;
for k=1:(n-1)
    k1 = ode(x(:,k),       u(:,k)); 
    k2 = ode(x(:,k)+h/2*k1,u(:,k));
    k3 = ode(x(:,k)+h/2*k2,u(:,k));
    k4 = ode(x(:,k)+h*k3,  u(:,k));
    opti.subject_to(x(:,k+1) == x(:,k) + h/6 * (k1 + 2*k2 + 2*k3 + k4));
end

% Path constraints:
opti.subject_to(x(:,1)==x_begin);
opti.subject_to(x(:,end)==x_final);
opti.subject_to(v_min <= x(3,:) <= v_max);
opti.subject_to(v_min <= x(4,:) <= v_max);
opti.subject_to(u_min <= u(1,:) <= u_max);
opti.subject_to(u_min <= u(2,:) <= u_max);
opti.subject_to(T >= 0);

% Obstacle constraints:
N = size(meas,1);
for i=1:n
    sum = 0;
    for k=1:N
        th  = meas(k,1);
        r   = meas(k,2);
        p   = [-r*sin(th);r*cos(th)];
        g   = getGaussianValue(x(1:2,i),p,sigma_x,sigma_y);
        sum = sum + g;
    end
    opti.subject_to((sum)<=G_hat);
end

% Initial guess:
opti.set_initial(T, T_init); % seconds
opti.set_initial(x, x_init);

% Objective function:
opti.minimize(T);
opti.solver('ipopt');

% Solve optimization problem:
sol = opti.solve();




% ***** PLOTS ***** %

% Extract solution:
T = sol.value(T);
X = sol.value(x);
U = sol.value(u);
t = linspace(0,T,n);

% Plot solution:
figure; hold all;
plot(X(1,:),X(2,:));
plot(-meas(:,2).*sin(meas(:,1)), meas(:,2).*cos(meas(:,1)),'x');
legend('Optimal trajectory','Rangefinder measurements');
axis equal;
title('Measurements and Optimal trajectory');

figure;
subplot(2,2,1); plot(t,X(3,:)); title('v_{x}(t)');
subplot(2,2,3); plot(t,X(4,:)); title('v_{y}(t)');
subplot(2,2,2); plot(t,U(1,:)); title('a_{x}(t)');
subplot(2,2,4); plot(t,U(2,:)); title('a_{y}(t)');

%% 

% To be able to reuse functions defined originally for use in the main
% script, make struct 'veh' that contains all important data that the
% vehicle (robot) has access to:

sit.vehNum          = 1;
sit.envNum          = 1;
sit.orientation     = 0;
sit.startState      = [];
sit.goalState	    = [];
sit.states          = {sit.startState};
sit.localGoals      = {};
sit.meas            = {};
sit.Init.T          = {5};
sit.Init.path       = {};
sit.Sol.X           = {};
sit.Sol.U           = {};
sit.Sol.T           = {};
sit.Sol.G           = {}; % solution G-value logger
sit.nNew            = {}; % MPC states advanced logger


veh.Map.N               = 100;
veh.Optim.sigma_x       = sigma_x;
veh.Optim.sigma_y       = sigma_y;
veh.Optim.G_hat         = G_hat;
veh.Sensor.horizon      = 5;
sit.meas{end+1}         = meas;
sit.states{end+1}       = x_begin;
sit.localGoals{end+1}   = x_final;
sit.Sol.X{end+1}        = X;
sit.Sol.U{end+1}        = U;
sit.Sol.T{end+1}        = T;

% Make robot local map:
sit = makeLocalMap(sit,veh);

% Expand rangefinder data with gaussians and plot:
sit = addGaussian(sit,veh);
%plotLocalMap(veh);

% Plot robot optimal trajectory in global map, together with the gaussians:
% veh = addPathToMap(veh);
plotLocalSolution(sit,veh,1)


% TEST THE SOLUTION SATISFIES OBSTACLE CONSTRAINT (OLD, DO NOT USE EXCEPT
% FOR CASE WITH ONLY ONE MEASUREMENT):
% fprintf('\n');
% for i = 1:size(veh.optim.X,2)
%    fprintf('Gaussian Value: %f \n',getGaussianValue(veh.optim.X(1:2,i),[-r*sin(th);r*cos(th)],sigma_x,sigma_y));
% end