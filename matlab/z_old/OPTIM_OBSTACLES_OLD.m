% Test of a simple time optimization of an acceleration-steered point mass
% with obstacles and multiple-shooting using Runge-Kutta integration.

% clear;
close all;
addpath('~/Downloads/casadi-matlabR2014a-v3.4.5/');

% Load previous solution from ipopt:
X_ipopt = load('X_ipopt.mat');

% Define ODE:
ode = @(x,u)[x(3); x(4); u(1); u(2)];

% States integration:
% h  = T/n;
% for k=1:(n-1)
%     k1 = ode(x(:,k),       u(:,k)); 
%     k2 = ode(x(:,k)+h/2*k1,u(:,k));
%     k3 = ode(x(:,k)+h/2*k2,u(:,k));
%     k4 = ode(x(:,k)+h*k3,  u(:,k));
%     opti.subject_to(x(:,k+1) == x(:,k) + h/6 * (k1 + 2*k2 + 2*k3 + k4));
% end

% States integration:
h = casadi.SX.sym('h');

x = casadi.SX.sym('x',4);
u = casadi.SX.sym('u',2);
k1 = ode(x,       u); 
k2 = ode(x+h/2*k1,u);
k3 = ode(x+h/2*k2,u);
k4 = ode(x+h*k3,  u);

xf = x+ h/6 * (k1 + 2*k2 + 2*k3 + k4);
F = casadi.Function('F',{x,u,h},{xf});

opti = casadi.Opti();

% Number of points used for multiple shooting:
n = 500;

T = opti.variable(1,1);
x = {};
u = {};
for i=1:n
 x{end+1} = opti.variable(4);
 u{end+1} = opti.variable(2);
end
x{end+1} = opti.variable(4);
x = [x{:}];
u = [u{:}];

% Initial and final positions + initial guess for time and states:
% (state is [x;y;v_x;v_y]):
x_begin = [0;0;0;0];
x_final = [0;7;0;0];
T_init  = 1;
x_init  = [linspace(0,x_final(1),n+1);0*linspace(0,x_final(2),n+1); ...
    zeros(1,n+1); zeros(1,n+1)];

% Make simple cases of rangefinder measurement data, called 'meas'.
% Structure of 'meas':
%       [theta1  r1;
%       [theta2  r2;
%       ...]
% (Every row represents a measurement)

%meas = [-pi/4 3];
%meas = [pi/6 3; -pi/4 5; -pi/3 3];
meas = [-pi/6 3; -pi/4 5; -pi/3 3];  % this one gives problems, even
                                     % when 'n' is made larger.
% meas = veh.Sensor.measurements;

% Gaussian parameters:
sigma_x = 0.2;
sigma_y = 0.2;

% Constraint values:
u_min = -2;
u_max = 2;
v_min = -10;
v_max = 10;
G_hat = 0.3;

opti.subject_to([F(x(:,1:end-1),u,T/n);u]==[x(:,2:end);u_max*ones(1,n);u_max*ones(1,n)]);

% Path constraints:
opti.subject_to(x(:,1)==x_begin);
opti.subject_to(x(:,end)==x_final);
opti.subject_to(v_min <= x(3,:) <= v_max);
opti.subject_to(v_min <= x(4,:) <= v_max);
opti.subject_to(u_min <= u(1,:) <= u_max);
opti.subject_to(u_min <= u(2,:) <= u_max);
opti.subject_to(T >= 0);
min_scale = 0.1;
opti.subject_to(-min_scale<=diff(x(1,:))<=min_scale);
opti.subject_to(-min_scale<=diff(x(2,:))<=min_scale);

% Obstacle constraints:
% N = size(meas,1);
% for i=1:n
%     sum = 0;
%     for k=1:N
%         th  = meas(k,1);
%         r   = meas(k,2);
%         p   = [-r*sin(th);r*cos(th)];
%         g   = getGaussianValue(x(1:2,i),p,sigma_x,sigma_y);
%         sum = sum + g;
%     end
%     % Som(1,i) = sum/N;
%     % opti.subject_to((sum/N)<=G_hat);
%     opti.subject_to((sum)<=G_hat);
% end

% Obstacle constraints:
pos = casadi.SX.sym('pos',2);

sum = 0;
for k=1:size(meas,1)
    th  = meas(k,1);
    r   = meas(k,2);
    p   = [-r*sin(th);r*cos(th)];
    g   = getGaussianValue(pos,p,sigma_x,sigma_y);
    sum = sum + g;
end

costf = casadi.Function('costf',{pos},{sum});

opti.subject_to(costf(x(1:2,:))<=G_hat);

% Initial guess:
opti.set_initial(T, X_ipopt.t); % seconds
opti.set_initial(x, X_ipopt.x);
opti.set_initial(u, X_ipopt.u);

% Objective function:
opti.minimize(T);
options = struct;
options.expand = true;
%options.qpsol = 'qrqp';
opti.solver('ipopt',options);

% Solve optimization problem:
sol = opti.solve();


% ***** PLOTS ***** %

% Extract solution:
T = sol.value(T);
X = sol.value(x);
U = sol.value(u);
t = linspace(0,T,n+1);

% Plot solution:
figure; hold all;
plot(X(1,:),X(2,:),'o-');
plot(-meas(:,2).*sin(meas(:,1)), meas(:,2).*cos(meas(:,1)),'x');
legend('Optimal trajectory','Rangefinder measurements');
axis equal;
title('Measurements and Optimal trajectory');

figure;
subplot(2,2,1); plot(t,X(3,:)); title('v_{x}(t)');
subplot(2,2,3); plot(t,X(4,:)); title('v_{y}(t)');
subplot(2,2,2); stairs(t,[U(1,:) nan]); title('a_{x}(t)');
subplot(2,2,4); stairs(t,[U(2,:) nan]); title('a_{y}(t)');

% To be able to reuse functions defined originally for use in the main
% script, make struct 'veh' that contains all important data that the
% vehicle (robot) has access to:
veh.Map.N               = 100;
veh.optim.sigma_x       = sigma_x;
veh.optim.sigma_y       = sigma_y;
veh.optim.G_hat         = G_hat;
veh.optim.x_final       = x_final;
veh.optim.X             = X;
veh.Sensor.horizon      = 5;
veh.Sensor.measurements = meas;

% Make robot local map:
veh = makeLocalMap(veh);
veh = addMeasurementsToMap(veh);

% Expand rangefinder data with gaussians and plot:
veh = addGaussian(veh);
%plotLocalMap(veh);

% Plot robot optimal trajectory in global map, together with the gaussians:
veh = addPathToMap(veh);
plotLocalSolution(veh);


% TEST THE SOLUTION SATISFIES OBSTACLE CONSTRAINT (OLD, DO NOT USE EXCEPT
% FOR CASE WITH ONLY ONE MEASUREMENT):
% fprintf('\n');
% for i = 1:size(veh.optim.X,2)
%    fprintf('Gaussian Value: %f \n',getGaussianValue(veh.optim.X(1:2,i),[-r*sin(th);r*cos(th)],sigma_x,sigma_y));
% end