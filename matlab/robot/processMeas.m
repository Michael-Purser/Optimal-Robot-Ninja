function sit = processMeas(sit,a)

xp = sit.states{end-1}(1:2);    % previous state
xc = sit.states{end}(1:2);        % current state
mtp = sit.meas_tilde{end};      % previous adapted measurements
mc = sit.meas{end};             % current measurements

% calculate R
R = a*norm(xc-xp,2);
fprintf('R-value: %f \n',R);

% delete current measurements closer than R to robot
mc = sortrows(mc,2);
c = 1;
while c==1
    if isempty(mc)==0 && mc(1,2) <= R
        mc(1,:) = [];
    else
        c = 0;
    end
end

% add previous measurements
if isempty(mtp)==0
    mtp = sortrows(mtp,2);
    c = 1;
    while c==1
        phi = mtp(1,1);
        r   = mtp(1,2);
        xm  = r*sin(phi);
        ym  = r*cos(phi);
        r2  = sqrt((xm-xc(1))^2+(ym-xc(2))^2);
        if r2 <= R
            mc = [atan2((xm-xc(1)),(ym-xc(2))) r2; mc];
            mtp(1,:) = [];
        else
            c = 0;
        end
    end
end

sit.meas_tilde{end+1} = mc;

end