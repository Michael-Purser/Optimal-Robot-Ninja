function MPC = sensor(MPC,veh,env)
% Function that simulates execution of the rangefinder and that returns the
% simulated measurements.
% The ouput vector only contains nonzero measurements, as zero distance 
% measurement is considered to be 'no obstacle detected'.

% get vehicle info:
pos         = [MPC.nav.currentState(1:2);1];
phi         = MPC.nav.currentState(3);

% get sensor info:
thetamax    = veh.sensor.thetamax;
h           = veh.sensor.horizon;
f           = veh.sensor.freq;
om          = veh.sensor.omega;
noiseamp    = veh.sensor.noiseamp;

% get relevant obstacle data:
P           = env.ObstDetect.pos;
Type        = env.ObstDetect.type;
Data        = env.ObstDetect.data;

% angular increment for rangefinder, and number of measurements:
dtheta      = om/f;
N           = floor(2*thetamax/dtheta);

% send out rangefinder beams and record measurements:
meas    = zeros(N,2);
for k = 1:N
    theta       = -thetamax + k*dtheta;
    meas(k,1)   = theta;
    v2          = [-h*sin(theta+phi); h*cos(theta+phi); 0];
    dist        = h*ones(1,size(Data,2));
    for i = 1:size(Data,2)
        type    = Type(i);
        if type == 1
            p       = P(:,i);
            v1      = p-pos;
            r       = Data(1,i);
            [I,d]   = distanceToCircleEdge(v1,v2,r);
        else
            p       = P(:,i);
            dx      = Data(1,i);
            dy      = Data(2,i);
            rho     = Data(3,i);
            C       = rectangleCorners(p,dx,dy,rho);
            [I,d]   = distanceToRectangleEdge(pos,v2,C);
        end
        % only add to dist if sensor can see obstacle; this will help in
        % finding smallest distance after loop:
        if I==1
            dist(i) = d;
        end
    end
    
    if min(dist)==h
        % if all elements of 'dist' have remained H, the  nothing has been
        % measured, so set to zero:
        meas(k,2) = 0;
    else
        % if 'Data' is empty, 'dist' is empty, so perform a check to avoid
        % errors:
        if (~isempty(dist))
            % smallest measured distance along beam is 'real' measurement
            % add noise to measurement:
            meas(k,2) = min(dist) + noiseamp*rand(1);
        end
    end
end

% remove all zero distance measurements:
meas = meas(all(meas,2),:);

% append result to vehicle struct:
MPC.nav.measurements = meas;

end