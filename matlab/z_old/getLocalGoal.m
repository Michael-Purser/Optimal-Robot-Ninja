function sit = getLocalGoal(sit)
% Transform robot goal position from global coordinates to local robot
% coordinates.

T = homTrans(sit.states{end}(3),[sit.states{end}(1:2);1]);
G = T\[sit.globalNotVisited(:,1);1];
sit.localGoals{end+1} = [G(1:2);sit.goalState(3)+sit.states{end}(3)];