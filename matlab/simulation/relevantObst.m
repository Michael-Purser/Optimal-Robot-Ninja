function env = relevantObst(MPC,veh,env)
% Function that gets the relevant obstacles around the vehicle given the
% vehicle position and the environment data.
% A relevant obstacle is an obstacle that is within range of the distance
% sensor.
% This is done so that the later simulation of rangefinder measurements can
% happen more efficiently

% extract data:
p  = [MPC.nav.currentState(1:2);1];
h  = veh.sensor.horizon;

% get nb of obstacles:
nObst = size(env.Obst.pos,2);

% make field in env struct for detected obstacles:
env.ObstDetect.pos  = [];
env.ObstDetect.type = [];
env.ObstDetect.data = [];

% iterate over obstacles and keep those within range:
for i=1:nObst
    pos  = env.Obst.pos(:,i);
    type = env.Obst.type(i);
    data = env.Obst.data(:,i);
    if type==1
        d = distanceToCircle(p,pos,data(1));
    else
        d = distanceToRectangle(p,pos,data(1),data(2),data(3));
    end
    if d<h
        env.ObstDetect.pos(:,end+1)  = pos;
        env.ObstDetect.type(end+1)   = type;
        env.ObstDetect.data(:,end+1) = data;
    end
end

end