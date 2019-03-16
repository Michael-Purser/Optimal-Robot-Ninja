function [map,N] = removeBorder(map,N)

map = map(2:(end-1),2:(end-1),:);

% Change N:
N = N-1;

end