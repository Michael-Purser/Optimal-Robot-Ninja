close all

x=linspace(0,1,10);
y = rand(10,1);

F=casadi.interpolant('r','bspline',{x},y,struct('algorithm','smooth_linear','smooth_linear_frac',0.2));
Fm=casadi.interpolant('r','bspline',{x},y);

xs = linspace(min(x),max(x),1000);
full(F(xs))

figure
hold on
plot(x,y,'o-')
plot(xs,full(F(xs)),'*-')
plot(xs,full(Fm(xs)),'*-')