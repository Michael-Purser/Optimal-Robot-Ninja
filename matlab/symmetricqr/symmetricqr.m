function D = symmetricqr(A,tol)
% Following algorithm 8.3.3
% careful: do not set 'tol' below machine accuracy!!

n = size(A,1);

% compute tridiagonalization
T = tridiag(A);

% initialize D matrix
D = T;
% TODO: get Q if desired

% iterations
fprintf('starting iterations \n');
q = 0;
while q<n
    
    % set to zero if tolerance reached
    for i=1:(n-1)
        if abs(D(i,i+1))<=(tol)*(D(i,i)+D(i+1,i+1))
            D(i,i+1) = 0;
            D(i+1,i) = 0;
        end
    end
    
    % find q and p: 
    p  = 0;
    q  = 0;
    sq = 0;
    sp = 0;
    
    for j=1:(n-1)
        if (D(n-j+1,n-j)==0 && sq==0)
            q = q+1;
        else
            sq = 1;
        end
        if (D(j,j+1)==0 && sp==0)
            p = p+1;
        else
            sp = 1;
        end
        if q == n-1
            q = n;
            p = 0;
        end
    end
    
%     fprintf('p and q: %i, %i \n',p,q);
    
    % get matrices D11, D22, D33
    D22 = D(p+1:n-q,p+1:n-q);
    D11 = [];
    D33 = [];
    if p>0
        D11 = D(1:p,1:p);
    end
    if q>0
        D33 = D(n-q+1:end,n-q+1:end);
    end
    
    % implicit symmetric QR step
    if q<n
        D22 = implicitstep(D22);
        D   = [D11 zeros(p,n-p);
               zeros(n-p-q,p) D22 zeros(n-p-q,q);
               zeros(q,n-q) D33];
        % TODO: calculate Q if desired
    end
    
end

end