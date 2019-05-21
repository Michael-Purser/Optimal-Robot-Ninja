function [map,n] = addBorder(map,n,a)
% Function that takes the empty local map and adds a border of cells around
% it containing values 'a'.

map = [a*ones(size(map,1),1) map a*ones(size(map,1),1)];
map = [a*ones(1,size(map,2));map;a*ones(1,size(map,2))];

% Because adding a border has shifted cell indices, increase N by 1 to keep
% it working:
n = n+1;

end