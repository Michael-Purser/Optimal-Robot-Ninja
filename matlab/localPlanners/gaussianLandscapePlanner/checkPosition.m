function G = checkPosition(meas,pos,sx,sy)
% Calculate the G-value of a given robot pôsition for a given set of
% measurements

G   = sum(gaussianValue(meas,pos,sx,sy));

end