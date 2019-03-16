function T = implicitstep(T)
% Following algorithm 8.3.2

n = size(T,1);
if n<2
    error('T must be at least 2x2 and tridiagonal');
end

d  = 0.5*(T(n-1,n-1)-T(n,n));
mu = T(n,n) - (T(n,n-1)^2)/(d+sign(d)*sqrt(d^2 + T(n,n-1)^2));
x  = T(1,1) - mu;
z  = T(2,1);
for k=1:(n-1)
    [c,s]  = givens(x,z);
    G      = eye(n);
    G(k,k) = c; G(k+1,k+1) = c;
    G(k,k+1) = s; G(k+1,k) = -s;
    T = G'*T*G;
    if k<(n-1)
        x = T(k+1,k);
        z = T(k+2,k);
    end
end