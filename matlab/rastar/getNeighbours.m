function neighbours = getNeighbours(current,map,threshold)

i = current(1);
j = current(2);
neighbours = [i-1,j-1;
              i,j-1;
              i+1,j-1;
              i-1,j;
              i+1,j;
              i-1,j+1;
              i,j+1;
              i+1,j+1];
k = 1;
while k<=size(neighbours,1)
    if map(neighbours(k,1),neighbours(k,2),1)>=threshold
        neighbours(k,:) = [];
    else
        k = k+1;
    end
end

end