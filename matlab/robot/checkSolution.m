function G = checkSolution(meas,x,sigma)
% For a given trajectory X, calculate the G-value in every point of the
% trajectory and store it for later examination.

G = zeros(1,size(x,2));
for i=1:size(x,2)
    G(i) = checkPosition(meas,x(:,i),sigma,sigma);
end

end