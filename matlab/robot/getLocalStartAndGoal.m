function MPC = getLocalStartAndGoal(MPC)

x           = MPC.nav.currentState;
plan        = MPC.nav.globalPlan.worldCoordinates;
R           = MPC.nav.opt.globalPlanR;
globalStart = MPC.nav.globalStart;
globalGoal  = MPC.nav.globalGoal;

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

% get the local goal orientation
% estimate this orientation by taking the angle of the line connecting the
% point before and after the point on the plan
% exceptions:
% if the local goal is the first plan point: in this case the orientation
% is the orientation of the global Start
% if the local goal is the last plan point: in this case the orientation is
% the orientation of the global Goal
if current_index == 1
    ori = globalStart(3);
elseif current_index == size(plan,1)
    ori = globalGoal(3);
else
    prevP = plan(current_index-1,:);
    nextP = plan(current_index+1,:);
    angle = atan2((nextP(2)-prevP(2)),(nextP(1)-prevP(1)));
    ori   = pi/2 - angle; % because of different definition of robot angle
end
ori = ori + x(3); % transform to local coordinate system

% transform goal position from global coordinates to local robot
% coordinates
T = homTrans(x(3),[x(1:2);1]);
G = T\[g';1];
MPC.nav.opt.goal = [G(1:2);ori];

% For now, robot always at center of local frame so start is always zero:
MPC.nav.opt.start = [0;0;0];

end