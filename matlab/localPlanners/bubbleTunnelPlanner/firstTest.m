% initialize variables:
opti = casadi.Opti();

% parameters:
n = 200;
xbeginp = [0;0;0];
xfinalp = [0;4;0];
uminp = -0.1;
umaxp = 0.1;
maxDistp = 0.1;
measp = 20*ones(100,2); % a lot of far away obstacles
Ghatp = 2;
sigmap = 0.1;
L = 0.2;
D = 2*ones(1,n+1);

% ode:
ode  = @(x,u)[0.5*(u(1)+u(2))*sin(x(3)); 0.5*(u(1)+u(2))*cos(x(3)); (2/L)*(u(2)-u(1))];

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
u    = opti.variable(2,n);
T    = opti.variable(1,n);

% multiple-shooting constraint
opti.subject_to(F(x(:,1:end-1),u,T./n)==x(:,2:end));

% state constraints
opti.subject_to(x(:,1)==xbeginp);
opti.subject_to(x(:,end)==xfinalp);
opti.subject_to(-maxDistp<=diff(x(1,:))<=maxDistp);
opti.subject_to(-maxDistp<=diff(x(2,:))<=maxDistp);

% velocity constraints
opti.subject_to(uminp <= u(1,:) <= umaxp);
opti.subject_to(uminp <= u(2,:) <= umaxp);

% time constraints
opti.subject_to(T >= 0);
opti.subject_to(T(2:end)==T(1:end-1));

% obstacle avoidance constraint
pos     = casadi.MX.sym('pos',2);
ds      = pos(1)-xbeginp(1);
costf   = casadi.Function('costf',{pos},{ds});
opti.subject_to(-D<=costf(x(1:2,:))<=D);

% objective:
opti.minimize(sum(T)/n);

% solver:
options.ipopt.print_level = 5;
opti.solver('ipopt',options);

% solve:
sol = opti.solve();

% extract solutions and display:
T = sol.value(T);
X = sol.value(x);
U = sol.value(u);

tvec = linspace(0,T(end),size(U,2));

figure;
plot(X(1,:),X(2,:)); axis equal;

figure;
hold all;
plot(tvec,U(1,:)); hold on;
plot(tvec,U(2,:));
legend('Uright','Uleft');

fprintf('Time: %f \n',T(end));