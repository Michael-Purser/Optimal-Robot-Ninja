function plotMap(map,dx,H,start,goals,path)

figure;

subplot(1,2,1);
hold all; 
for i = 1:size(map,1)
   for j = 1:size(map,2)
       if map(i,j)>=1
           plot(-dx/2-H+i*dx,-dx/2-H+j*dx, 'k.');
       end
   end
end
plot(start(1),start(2),'b.');
for k = 1:size(goals,1)
    plot(goals(k,1),goals(k,2), 'r.');
end
axis equal; 
axis(1.1*[-H H -H H]); 
title('Map with obstacles');

subplot(1,2,2);
hold all; 
for i = 1:size(map,1)
   for j = 1:size(map,2)
       if map(i,j)>=1
           plot(-dx/2-H+i*dx,-dx/2-H+j*dx, 'k.');
       end
   end
end
plot(start(1),start(2),'b.');
plot(path(:,1),path(:,2),'b.-');
for k = 1:size(goals,1)
    plot(goals(k,1),goals(k,2), 'r.');
end
axis equal;
lim = max(max(abs(path(:))),H);
axis(1.1*[-lim lim -lim lim]); 
title('Map with obstacles and path');

end