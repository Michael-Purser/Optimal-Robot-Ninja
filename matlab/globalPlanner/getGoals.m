function goals = getGoals(map,goal_abs,H)
% Function that returns the goals on the edge(s) of the map for when the
% actual goal is outside of the map.
% We assume that the map has been preprocessed by adding layers 1 and 2.
% Note that in some cases the returned list will have a duplicates, namely
% the corner cell of the map. This however does not pose any major problem.

goals = [];
A = 2*ones(1,size(map,1)-2);
B = (size(map,1)-1)*ones(1,size(map,1)-2);
C = linspace(2,size(map,1)-1,size(map,1)-2);

if goal_abs(1)<=-H
    added = [A;C]';
    goals = [goals;added];
end
if goal_abs(1)>=H
    added = [B;C]';
    goals = [goals;added];
end
if goal_abs(2)<=-H
    added = [C;A]';
    goals = [goals;added];
end
if goal_abs(2)>=H
    added = [C;B]';
    goals = [goals;added];
end

end