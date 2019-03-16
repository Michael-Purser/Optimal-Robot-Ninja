function A = tridiag(A)
% Following algorithm 8.3.1

n = size(A,1);

for k=1:(n-2)
    [b,v]    = house(A((k+1):n,k));
    p        = b*A((k+1):n,(k+1):n)*v;
    w        = p - (b*p'*(v/2))*v;
    A(k+1,k) = norm(A((k+1):n,k),2);
    A((k+2):end,k) = 0; % added, not in book
    A(k,k+1) = A(k+1,k);
    A(k,(k+2):end) = 0; % added, not in book
    A((k+1):n,(k+1):n) = A((k+1):n,(k+1):n) - v*w' - w*v';
end

end