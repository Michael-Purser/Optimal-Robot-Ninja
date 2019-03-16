function env = createEnv(env,veh)
% Create a simple environment 'env', containing circular and
% rectangular obstacles.
% Also check that the robot is not initially inside an obstacle.
    
% Get the limits of the environment:
nObst   = size(env.Obst.Pos,2);
low     = zeros(1,nObst);
high    = zeros(1,nObst);
left    = zeros(1,nObst);
right   = zeros(1,nObst);
for i=1:nObst
    type = env.Obst.Type(i);
    data = env.Obst.Data(:,i);
    pos  = env.Obst.Pos(:,i);
    if type==1
        R       = data(1);
        left(i) = pos(1)-R;
        right(i)= pos(1)+R;
        low(i)  = pos(2)-R;
        high(i) = pos(2)+R;
    else
        dx      = data(1);
        dy      = data(2);
        theta   = data(3);
        C = getRectangleCorners(pos,theta,dx,dy);
        left(i) = min(C(1,:));
        right(i)= max(C(1,:));
        low(i)  = min(C(2,:));
        high(i) = max(C(2,:));
    end
end

robotPos    = veh.pos;
left(end+1) = robotPos(1);
right(end+1)= robotPos(1);
low(end+1)  = robotPos(2);
high(end+1) = robotPos(2);

x_min   = min(left);
x_max   = max(right);
y_min   = min(low);
y_max   = max(high);

lim  = [x_min x_max y_min y_max];

% Append return struct:
env.Map.lim = lim;
    
end