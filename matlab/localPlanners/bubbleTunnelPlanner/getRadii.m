function localPlanner = getRadii(localPlanner)

plan    = [localPlanner.params.xglobalx,localPlanner.params.xglobaly];
obst    = localPlanner.obstacleData;
maxDist = localPlanner.params.maxR;

radii = zeros(1,size(plan,1));

for k=1:size(plan,1)
    
    posOnPath = plan(k,:);
    closestObst = 1;
    smallestDist = 1000; % some big number
    
   for l=1:size(obst,1)
       
       dist = norm(posOnPath-obst(l,:));
       if dist<smallestDist
           smallestDist = dist;
           closestObst = l;
       end
       
   end
   
   % append smallest found radius to list of radii
   if smallestDist>maxDist
       smallestDist = maxDist;
   end
   radii(k) = smallestDist;
    
end

localPlanner.params.radii = radii;
