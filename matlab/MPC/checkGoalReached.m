function MPC = checkGoalReached(MPC)
% Function that checks if the goal is reached within tolerance
% If not, a message is displayed showing the remaining euclidian distance
% to goal.

currentState    = MPC.nav.currentState;
globalGoal      = MPC.nav.globalGoal;

distanceToGoal  = norm(currentState(1:2)-globalGoal(1:2));

if distanceToGoal<MPC.nav.goalTolerance
    fprintf(2,'\t Vehicle localized to be within tolerance of final goal! \n');
    fprintf(2,'\t STOPPED \n');
    fprintf(2,'\n');
    MPC.nav.goalReached = true;
else
    fprintf('\t Distance to goal: %f [m] \n',distanceToGoal);
end