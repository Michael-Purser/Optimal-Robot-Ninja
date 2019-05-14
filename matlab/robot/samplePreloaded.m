function MPC = samplePreloaded(MPC,env)
% Function that samples the preloaded obstacles into equally spaced
% discrete points, to be treated as 'sensor measurements gathered in
% advance'

% get the equivalent deltaL value
deltaL = 0.01;

% sample the obstacles
preloaded = [];
for i=1:size(env.mapped,2)
    type = env.mapped{i}.type;
    if strcmp(type,'circle')
        center  = env.mapped{i}.center;
        R       = env.mapped{i}.radius;
        N       = ceil(2*pi*R/deltaL);
        meas    = zeros(N,2);
        for j=1:N
           meas(j,:) = [R*cos(j*2*pi/N) R*sin(j*2*pi/N)];
        end
        T = homTrans(0,center);
        for j=1:N
           A = T*[meas(j,:)';1];
           meas(j,:) = A(1:2);
        end
        
    elseif strcmp(type,'rectangle')
        
    end
    preloaded = [preloaded;meas];
end

MPC.nav.obstacleData.preloaded = preloaded;

end