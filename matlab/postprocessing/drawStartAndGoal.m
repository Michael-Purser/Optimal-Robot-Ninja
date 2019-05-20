function drawStartAndGoal(start,goal)
% Function that draws MPC start and goal states in world coordinate frame.

plot(start(1),start(2),'ko','LineWidth',1.5);
quiver(start(1),start(2),sin(start(3)),cos(start(3)),'k','LineWidth',1.5);

plot(goal(1),goal(2),'kx','LineWidth',1.5);
quiver(goal(1),goal(2),sin(goal(3)),cos(goal(3)),'k','LineWidth',1.5);