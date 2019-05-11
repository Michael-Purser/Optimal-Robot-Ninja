function G = checkPosition(meas,pos,sx,sy)
% Calculate the G-value of a given robot pôsition for a given set of
% measurements


th  = meas(:,1);
r   = meas(:,2);
p   = [-r.*sin(th) r.*cos(th)];
G   = sum(gaussianValue(p,pos,sx,sy));

end