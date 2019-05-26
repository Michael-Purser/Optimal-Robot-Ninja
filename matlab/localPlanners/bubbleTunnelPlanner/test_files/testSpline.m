x = sort(rand(100,1));
y = sort(rand(size(x)));
f=fit(x,y,'smoothingspline');

plot(x,y); hold on;
plot(x,f(x));


