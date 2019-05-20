function drawStates(states)
% Function that draws the vehicle states in world coordinate frame.

for k=1:size(states,2)
    plot(states{k}(1),states{k}(2),'k.','MarkerSize',6,'LineWidth',1.5);
end