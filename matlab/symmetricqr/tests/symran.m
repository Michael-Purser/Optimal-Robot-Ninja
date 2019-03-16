function A = symran(N,alpha)
% FOR TESTING
% Makes a random symmetric matrix of size NxN, with factor alpha for the
% size of the elements.

d = alpha*rand(N,1); % The diagonal values
t = triu(bsxfun(@min,d,d.').*rand(N),1); % The upper trianglar random values
A = diag(d)+t+t.'; % Put them together in a symmetric matrix