plan = globalPlanner.worldCoordinates;
plan2 = plan;
plan2(:,1) = sgolayfilt(plan(:,1),3,7);
plan2(:,2) = sgolayfilt(plan(:,2),3,7);

plotGlobalPlan(env,globalPlanner); hold on;
plot(plan2(:,1),plan2(:,2),'r','Linewidth',1.4);