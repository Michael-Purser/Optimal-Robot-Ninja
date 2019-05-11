%%
clear;
clc;

opti = casadi.Opti();

x = opti.variable(1);
xinit = opti.parameter(1);
opti.minimize(sin(x));
opti.solver('ipopt');

opti.subject_to(x<xinit);

opti.set_value(xinit,3);

opti.set_initial(x,-9);

F=opti.to_function('F',{x},{x});

%%
clear;
clc;
x = casadi.SX.sym('x',1);
y = casadi.SX.sym('y',1);
z = casadi.SX.sym('z',1);
y = 2*x;
z = 3*x;
F = casadi.Function('F',{x},{y,z});