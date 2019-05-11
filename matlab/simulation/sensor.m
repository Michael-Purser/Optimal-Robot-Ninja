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

% angular increment for rangefinder, and number of measurements:
dtheta      = om/f;
N           = floor(2*thetamax/dtheta);

% send out rangefinder beams and record measurements:
meas    = zeros(N,2);
for k = 1:N
    theta       = -thetamax + k*dtheta;
    meas(k,1)   = theta;
    v2          = [-h*sin(theta+phi); h*cos(theta+phi); 0];
    dist        = h*ones(1,size(env.measured,2));
    for i = 1:size(env.measured,2)
        type    = env.measured{i}.type;
        if strcmp(type,'circle')
            p       = env.measured{i}.center;
            v1      = p-pos;
            r       = env.measured{i}.radius;
            [I,d]   = distanceToCircleEdge(v1,v2,r);
        elseif strcmp(type,'rectangle')
            p       = env.measured{i}.center;
            dx      = 0.5*env.measured{i}.width;
            dy      = 0.5*env.measured{i}.height;
            rho     = env.measured{i}.orientation;
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
        % if obstacle list is empty, 'dist' is empty, so perform a check to avoid
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