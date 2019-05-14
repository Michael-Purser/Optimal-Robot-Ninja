function MPC = getLocalStartAndGoal(MPC)

x       = MPC.nav.currentState;
plan    = MPC.nav.globalPlan.worldCoordinates;
R       = MPC.nav.opt.globalPlanR;

% Iterate over global plan and get last point on plan
% if no point of plan is further than global plan horizon, get last one in
% the list
% also keep index of last found local goal; the starting value of i and the
% fact that i cannot decrease guarantees that the local goal will never
% 'regress' along the global path.
% THIS METHOD ONLY WORKS AS LONG AS THE GLOBAL PATH STAYS WITHIN THE
% VEHICLE VIEW RADIUS!!

last_index = MPC.nav.lastIndex;
current_index = last_index;
found = 0;
i = last_index;
while found==0
    % find the first point beyond range
    if norm(x(1:2)-plan(i,:)')>R
        found = 1;
        current_index = i;
    end
    i = i+1;
    if i==size(plan,1)
        % if no goal found in the entire plan, assume the global goal is
        % within view and set the last plan element as local goal
        found = 1;
        current_index = size(plan,1);
    end
end

MPC.nav.lastIndex = current_index;
g = plan(current_index,:);

% transform goal position from global coordinates to local robot
% coordinates
T = homTrans(x(3),[x(1:2);1]);
G = T\[g';1];
MPC.nav.opt.goal = [G(1:2);MPC.nav.globalGoal(3)+x(3)];

% For now, robot always at center of local frame so start is always zero:
MPC.nav.opt.start = [0;0;0];

end