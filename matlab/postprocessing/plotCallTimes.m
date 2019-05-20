function plotCallTimes()
% Function that plots the recorded execution times of the MPC loop for
% different solver configurations
% Input:    filename        file containing the execution times for a 
%                           solver configuration, stored in vectors. 
%                           All these tests be performed on the same
%                           situation and with the same optimization and
%                           MPC parameters!
%                           The responsibility of building this file
%                           is left to the user.

% Load the file contents
load('timesIPopt.mat');
load('timesSQP1.mat');
load('timesSQP2.mat');
load('timesSQPAll.mat');

% Plot the vectors on one graph:
figure;
plot(linspace(1,size(ipopt,2),size(ipopt,2)), ipopt, '-.'); hold on;
plot(linspace(1,size(SQPAll,2),size(SQPAll,2)), SQPAll, '-.'); hold on;
plot(linspace(1,size(SQP2,2),size(SQP2,2)), SQP2, '-.'); hold on;
plot(linspace(1,size(SQP1,2),size(SQP1,2)),SQP1, '-.'); hold on;
legend('ipopt','SQP full','SQP 2 iterations','SQP 1 iteration');
title('Execution times over MPC iterations');
xlabel('iteration [-]'); ylabel('CPU time [s]');
