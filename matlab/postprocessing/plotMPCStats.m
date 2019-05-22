function plotMPCStats(log)

fprintf('Plotting MPC stats \n');

% plot n_new and euclidian distance between successive MPC states:
figure;

subplot(1,2,1); hold all;
for k = 1:size(log.m,2)
    plot(k,log.m{k},'b.-'); 
    xlabel('iteration [-]'); ylabel('n_{new} [-]');
    titlestr = {'n_{new} over successive','MPC iterations'};
    title(titlestr);
end

subplot(1,2,2); hold all;
for k = 1:size(log.states,2)-1
    s1 = log.states{k}(1:2);
    s2 = log.states{k+1}(1:2);
    d = norm(s1-s2);
    plot(k,d,'b.-'); 
    xlabel('iteration [-]'); ylabel('d [m]');
    titlestr = {'Euclidian distance between','successive MPC states'};
    title(titlestr);
end

% plot wall times
figure;
hold all;
for k = 1:size(log.localPlanners,2)
    plot(k,log.localPlanners{k}.sol.stats.t_wall_solver,'b.-'); 
    xlabel('iteration [-]'); ylabel('Solver CPU time [s]');
    titlestr = {'Solver CPU time (wall time) over','successive MPC iterations'};
    title(titlestr);
end

% plot extra stats from local planner (varies per planner)
plotLocalStats(log);