map = MPC.nav.map.inflated;
Nw  = MPC.nav.map.Nw;
Nh  = MPC.nav.map.Nh;
plan = MPC.nav.globalPlan.gridCoordinates;
figure
hold all;

% plot grid map
for i=1:size(map,1)
    for j=1:size(map,2)
        if map(i,j)>0
            % plot a filled square
            x = [i-0.5 i-0.5 i+0.5 i+0.5];
            y = [j-0.5 j+0.5 j+0.5 j-0.5];
            fill(x,y,'k');
        end
    end
end

for k=1:size(plan,1)
    % plot a filled square
    i = plan(k,1);
    j = plan(k,2);
    x = [i-0.5 i-0.5 i+0.5 i+0.5];
    y = [j-0.5 j+0.5 j+0.5 j-0.5];
    fill(x,y,'r','LineStyle','none');
end

% plot(plan(:,1),plan(:,2),'r','Linewidth');
axis equal;
axis([0 Nw 0 Nh]);
grid on;
titlestr = {'Known obstacle grid map','and global plan in grid coordinates'};
title(titlestr);