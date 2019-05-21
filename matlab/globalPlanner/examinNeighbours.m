function [map,open] = examinNeighbours(current,neighbours,map,goals,open,tBreak)

for k = 1:size(neighbours,1)
    neighbour = neighbours(k,:);
    if map(neighbour(1), neighbour(2),2)==inf
        g = map(current(1),current(2),2) + norm(current-neighbour);
        f_scores = inf*ones(size(goals,1),1);
        for n=1:size(goals,1)
            goal = goals(n,:);
            f_scores(n) = g+tBreak*norm(goal-neighbour);
        end
        f = min(f_scores);
        open(end+1,:) = [neighbour(1),neighbour(2),g,f];
        map(neighbour(1), neighbour(2),2) = g;
    end
end

end