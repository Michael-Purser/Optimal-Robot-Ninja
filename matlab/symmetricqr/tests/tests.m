%% Test 1: Example 8.3.3

A   = [1 2 0 0;2 3 4 0;0 4 5 6;0 0 6 7];
tol = 1e-9;
D   = symmetricqr(A,tol);

lambda = diag(D);
fprintf('\n')
fprintf('TEST 1: Result for example 8.3.3: \n')
disp(sort(lambda));

%% Test 2: random symmetric 5x5 matrix

A   = symran(5,1);
tol = 1e-9;
D   = symmetricqr(A,tol);
lambda = diag(D);

% compute using matlab 'eig' for comparison:
lambda2 = eig(A);

fprintf('\n')
fprintf('TEST 2: Result for random 5x5 matrix: \n')
fprintf('    Result method: \n')
disp(sort(lambda));
fprintf('    Result matlab built-in method: \n')
disp(lambda2);


%% Test 3: random symmetric 10x10 matrix

A   = symran(10,1);
tol = 1e-3;
D   = symmetricqr(A,tol);
lambda = diag(D);

% compute using matlab 'eig' for comparison:
lambda2 = eig(A);

fprintf('\n')
fprintf('TEST 3: Result for random 10x10 matrix: \n')
fprintf('    Result method: \n')
disp(sort(lambda));
fprintf('    Result matlab built-in method: \n')
disp(lambda2);