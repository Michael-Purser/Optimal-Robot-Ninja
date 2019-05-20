function drawLocalGoal(goal)
% Function that draws the local goal in world coordinates

plot(goal(1),goal(2),'or','LineWidth',1.5);
quiver(goal(1),goal(2),sin(goal(3)),cos(goal(3)),'k','LineWidth',1.5);