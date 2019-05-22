function grid_coords = getIndices(map,n,dx,coords,threshold)

grid_coords=zeros(size(coords));
for k = 1:size(coords,1)
    
    entry = coords(k,:);
    
    % Get cell indices:
    nx      = n + 1 + floor(entry(1)/dx);
    ny      = n + 1 + floor(entry(2)/dx);

    % Check that given position is not in an obstacle. If it's not, add to map:
    if map(nx,ny)>=threshold
        error('One of the given positions seems to be in an obstacle! Examining adjacent cells \n');
        neighbours = getNeighbours([nx,ny],map,threshold);
        if size(neighbours,1)==0
            error('All neighbours are in obstacle! Aborting \n');
        else
            grid_coords = neighbours(1,:);
        end
    else
        grid_coords(k,:) = [nx,ny];
    end
    
end