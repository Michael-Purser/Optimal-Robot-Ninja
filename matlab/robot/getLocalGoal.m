function sit = getLocalGoal(sit,veh)

gv      = sit.globalVisited;
gnv     = sit.globalNotVisited;
x       = sit.states{end}(1:2);
H       = veh.Sensor.horizon;
f       = sit.viewFactor;

% if a new waypoint comes into view, set it as new goal and move the old
% goal to the visited list; if not, keep the first entry of unvisited as
% local goal:
while norm(gnv(:,1)-x) <= f*H && size(gnv,2)>1
    gv = [gv gnv(:,1)];
    gnv(:,1) = [];
    fprintf(2,'********************* NEW GOAL ********************** \n');
end

% transform goal position from global coordinates to local robot
% coordinates
T = homTrans(sit.states{end}(3),[sit.states{end}(1:2);1]);
G = T\[gnv(:,1);1];
sit.globalVisited = gv;
sit.globalNotVisited = gnv;
sit.localGoals{end+1} = [G(1:2);sit.goalState(3)+sit.states{end}(3)];

end