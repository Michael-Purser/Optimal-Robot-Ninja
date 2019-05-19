function plotGlobalPlan(MPC,env)
% Function that plots the environment and the calculated global plan.

globalPlan = MPC.nav.globalPlan.worldCoordinates;
plotEnv(env,MPC);
hold on;
plot(globalPlan(:,1),globalPlan(:,2),'g'); 
axis equal;
title('Global plan');

end