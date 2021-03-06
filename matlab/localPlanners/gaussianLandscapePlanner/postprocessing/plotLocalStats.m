function plotLocalStats(log)
% Function that calls the local planner obstacle extra stats plotting method.
% Acts as a 'bridge' between the general MPC plotting methods and the local
% planner displaying methods.
% The plots are generated on top of the ones generated by the standard
% MPC stats displaying method.

log = getSolutionGValues(log);
plotMaxG(log);