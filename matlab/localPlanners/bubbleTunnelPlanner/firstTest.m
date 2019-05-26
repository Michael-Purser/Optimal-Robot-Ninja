addpath('~/Downloads/casadi/install/matlab/');

% initialize variables:
opti = casadi.Opti();

% parameters:
n           = 100;
uminp       = -0.2;
umaxp       = 0.2;
aminp       = -0.2;
amaxp       = 0.2;
jminp       = -0.5;
jmaxp       = 0.5;
omminp      = -pi;
ommaxp      = pi;
betap       = 3;
xg          = localPlanner.params.xglobalx';
yg          = localPlanner.params.xglobaly';
radii       = localPlanner.params.radii;
xbeginp     = localPlanner.params.start;
xfinalp     = localPlanner.params.goal;
ubeginp     = localPlanner.params.startVelocity;
L           = 0.4;

% ode:
ode  = @(x,u)[0.5*(u(1)+u(2))*sin(x(3)); 0.5*(u(1)+u(2))*cos(x(3)); (2/L)*(u(2)-u(1))];

% [globalPlanner,localPlanner] = getLocalStartAndGoal(MPC,globalPlanner,localPlanner);

% make the max dist value:
maxDist = betap*norm(xbeginp(1:2)-xfinalp(1:2))/n;

% parametric global plan
% xgx = @(t)-cos(t);
% xgy = @(t)sin(t);
% xg = [0*ones(1,20) linspace(0,5,20)];
% yg = [linspace(0,5,20) linspace(5,7,20)];
% xg = globalPlanner.worldCoordinates(:,1)';
% yg = globalPlanner.worldCoordinates(:,2)';
% xg = globalPlanner.worldCoordinates(1:globalPlanner.lastIndex,1)';
% yg = globalPlanner.worldCoordinates(1:globalPlanner.lastIndex,2)';
xgx=casadi.interpolant('r','bspline',{linspace(0,1,size(xg,2))},xg,...
    struct('algorithm','smooth_linear','smooth_linear_frac',0.2));
xgy=casadi.interpolant('r','bspline',{linspace(0,1,size(yg,2))},yg,...
    struct('algorithm','smooth_linear','smooth_linear_frac',0.2));

% xbeginp = [full(xgx(0));full(xgy(0));0];
% xfinalp = [full(xgx(1));full(xgy(1));pi/4];

% parametric obstacles
    % make some fake obstacle radii
%      radii = 0.1*[2 2 10 10 12 10 1 0.5];
    % fit a spline
%     f=fit(linspace(0,1,size(radii,2))',radii','smoothingspline');
r=casadi.interpolant('r','bspline',{linspace(0,1,size(radii,2))},radii,...
    struct('algorithm','smooth_linear','smooth_linear_frac',0.2));
    
% R = @(t)f(t);
% R = @(t)(0.05*(1+2*t));
% t       = casadi.MX.sym('s',1);
% F       = getFValue(f,t);
% R       = casadi.Function('R',{t},{F});

% initial values
theta_init = linspace(xbeginp(3),xfinalp(3),n+1);
x_init  = [linspace(xbeginp(1),xfinalp(1),n+1);linspace(xbeginp(2),xfinalp(2),n+1); ...
    theta_init];
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
opti.subject_to(-maxDist<=diff(x(1,:))<=maxDist);
opti.subject_to(-maxDist<=diff(x(2,:))<=maxDist);

% velocity constraints
opti.subject_to(uminp <= u(1,:) <= umaxp);
opti.subject_to(uminp <= u(2,:) <= umaxp);
opti.subject_to(u(:,1) == ubeginp);
opti.subject_to(u(1,:)+u(2,:) >= 0);

% omega constraint
opti.subject_to(omminp <= (1/L)*(u(2,:)-u(1,:)) <= ommaxp);

% acceleration constraints
opti.subject_to(aminp <= n/T*u(1,2:end)-n/T*u(1,1:end-1) <= amaxp);
opti.subject_to(aminp <= n/T*u(2,2:end)-n/T*u(2,1:end-1) <= amaxp);

% jerk constraints
opti.subject_to(jminp <= ((n/T)^2)*u(1,3:end)-2*((n/T)^2)*u(1,2:end-1)+...
    ((n/T)^2)*u(1,1:end-2) <= jmaxp);
opti.subject_to(jminp <= ((n/T)^2)*u(2,3:end)-2*((n/T)^2)*u(2,2:end-1)+...
    ((n/T)^2)*u(2,1:end-2) <= jmaxp);

% time constraints
opti.subject_to(T >= 0);

% obstacle avoidance 'tunnel' constraint
opti.subject_to(((x(1,:)-xgx(s(:))').^2 + (x(2,:)-xgy(s(:))').^2)<=r(s(:)').^2);

opti.subject_to(s(1) == 0);
opti.subject_to(s(n+1) == 1);
opti.subject_to(0<=s(:)<=1);
opti.subject_to(diff(s(:))<=1/n);

opti.set_initial(s,linspace(0,1,n+1));
opti.set_initial(x,x_init);
opti.set_initial(u,u_init);
opti.set_initial(T,T_init);

% objective:
opti.minimize(T);

% solver:
options.ipopt.print_level = 5;
opti.solver('ipopt',options);

% solve:
sol = opti.solve();

% extract solutions and display:
T = sol.value(T);
X = sol.value(x);
S = sol.value(s);
U = sol.value(u);

tvec = linspace(0,T(end),size(U,2));
arc = 0:0.05:2*pi;

figure;
hold all;
plotEnv(env);
for k=1:size(S,2)
    x_circle = full(xgx(S(k)))+full(r(S(k)))*cos(arc);
    y_circle = full(xgy(S(k)))+full(r(S(k)))*sin(arc);
    fill(x_circle, y_circle,'g','LineStyle','none');
end
plot(X(1,:),X(2,:),'.-','Linewidth',1.5);
plot(full(xgx(S(:))),full(xgy(S(:))));
axis equal;

figure;
hold all;
plot(tvec,U(1,:)); hold on;
plot(tvec,U(2,:));
legend('Uright','Uleft');

fprintf('Time: %f \n',T(end));