function [current,open] = getRemoveCurrent(open)

[~,I] = min(open(:,4));
current = [open(I,1),open(I,2)];
open(I,:) = [];

end