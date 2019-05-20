function [A,J,dT] = getAccelerationAndJerk(U,T,horizon)
% Given velocity profile U and time vector T, reconstruct accelerations and
% jerks using finite differences

% time difference
dT      = T(end)/horizon;

% reconstruct accelerations
A       = zeros(2,size(U,2)-1);
A(1,:)  = diff(U(1,:))/dT;
A(2,:)  = diff(U(2,:))/dT;

% reconstruct jerks
J       = zeros(2,size(U,2)-2);
J(1,:)  = diff(A(1,:))/dT;
J(2,:)  = diff(A(2,:))/dT;