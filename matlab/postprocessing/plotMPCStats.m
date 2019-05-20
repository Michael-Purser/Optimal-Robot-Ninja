function plotMPCStats(MPC)

% plot max(G) in all iterations:
figure;
hold all;
for k = 1:size(MPC.log.opts,2)
    plot(k,max(MPC.log.opts{k}.sol.Gvalues),'b.-'); 
    xlabel('iteration [-]'); ylabel('max(G) [-]');
    titlestr = {'max(G) over successive MPC iterations','(original measurements)'};
    title(titlestr);
end
    
% plot n_new and euclidian distance between successive MPC states:
figure;

subplot(1,2,1); hold all;
for k = 1:size(MPC.log.opts,2)
    plot(k,MPC.log.m{k},'b.-'); 
    xlabel('iteration [-]'); ylabel('n_{new} [-]');
    titlestr = {'n_{new} over successive','MPC iterations'};
    title(titlestr);
end

subplot(1,2,2); hold all;
for k = 1:size(MPC.log.states,2)-1
    s1 = MPC.log.states{k}(1:2);
    s2 = MPC.log.states{k+1}(1:2);
    d = norm(s1-s2);
    plot(k,d,'b.-'); 
    xlabel('iteration [-]'); ylabel('d [m]');
    titlestr = {'Euclidian distance between','successive MPC states'};
    title(titlestr);
end

% Plot wall times
figure;
hold all;
for k = 1:size(MPC.log.opts,2)
    plot(k,MPC.log.opts{k}.sol.stats.t_wall_solver,'b.-'); 
    xlabel('iteration [-]'); ylabel('Solver CPU time [s]');
    titlestr = {'Solver CPU time (wall time) over','successive MPC iterations'};
    title(titlestr);
end