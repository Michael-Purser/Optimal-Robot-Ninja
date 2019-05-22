function plotMaxG(log)
% function that plots max(G) over all MPC iterations
% only do it for grid measurements too if the option to use grid measurements
% has been set to true
% ASSUMPTION: variable withLocalGrid does not change over MPC execution
% (eg if the user decides to use the grid system it will do so for the
% whole run).

% get data
MPC_runs            = size(log.localPlanners,2);
withLocalGrid       = log.localPlanners{1}.withLocalGrid;

% plot max(G) over all iterations

if withLocalGrid
    
    figure;
    subplot(1,2,1); hold all;
    for k = 1:MPC_runs
        plot(k,max(log.localPlanners{k}.post.GValuesOrig),'b.-'); 
        xlabel('iteration [-]'); ylabel('max(G) [-]');
        titlestr = {'max(G) over successive MPC iterations','(original measurements)'};
        title(titlestr);
    end

    subplot(1,2,2); hold all;
    for k = 1:MPC_runs
        plot(k,max(log.localPlanners{k}.post.GValuesGrid),'b.-'); 
        xlabel('iteration [-]'); ylabel('max(G) [-]');
        titlestr = {'max(G) over successive MPC iterations','(grid-fitted measurements)'};
        title(titlestr);
    end
    
else
    fprintf(2,'WARNING: variable withLocalGrid set to false, only printing max(G) values for original measurements')
    figure;
    subplot(1,2,1); hold all;
    for k = 1:MPC_runs
        plot(k,max(log.localPlanners{k}.post.GValuesOrig),'b.-'); 
        xlabel('iteration [-]'); ylabel('max(G) [-]');
        titlestr = {'max(G) over successive MPC iterations','(original measurements)'};
        title(titlestr);
    end
    
end