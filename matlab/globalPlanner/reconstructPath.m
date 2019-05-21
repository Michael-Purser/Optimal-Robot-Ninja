function path = reconstructPath(goal,map,G_hat)

path = goal;
counter = 1;
% display(map(52,52,2));
while map(path(end,1),path(end,2),2)~=0
    current = path(end,:);
    neighbours = getNeighbours(current,map,G_hat);
    next = current;
    for k=1:size(neighbours,1)
        if map(neighbours(k,1),neighbours(k,2),2)<map(next(1),next(2),2)
            next = neighbours(k,:);
        end
    end
    path(end+1,:) = next;
    if path(end,:)==path(end-1,:)
        error('I got stuck!');
    end
    % fprintf('Iteration: %i, cell: [%i,%i] \n',counter,next(1),next(2));
    counter = counter + 1;
end


end