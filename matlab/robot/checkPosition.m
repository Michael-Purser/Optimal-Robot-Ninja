function G = checkPosition(meas,pos,sx,sy)
% Calculate the G-value of a given robot pôsition for a given set of
% measurements

G = 0;
for k=1:size(meas,1)
    th  = meas(k,1);
    r   = meas(k,2);
    p   = [-r*sin(th);r*cos(th)];
    g   = gaussianValue(pos,p,sx,sy);
    G = G + g;
end

end