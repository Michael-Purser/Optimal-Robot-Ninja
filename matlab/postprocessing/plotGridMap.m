function plotGridMap(MPC)
% Plots the discrete map with the preloaded obstacles used by the global
% planner

mapValues   = MPC.nav.map.values;
mapInflated = MPC.nav.map.inflated;
Nw          = MPC.nav.map.Nw;
Nh          = MPC.nav.map.Nh;
plan        = MPC.nav.globalPlan.gridCoordinates;
figure
hold all;

% plot grid map
for i=1:size(mapValues,1)
    for j=1:size(mapValues,2)
        if mapValues(i,j)>0
            % plot a filled translucent square
            x = [i-0.5 i-0.5 i+0.5 i+0.5];
            y = [j-0.5 j+0.5 j+0.5 j-0.5];
            fill(x,y,'k','LineStyle','none');
        end
    end
end

% plot inflated grid map
for i=1:size(mapInflated,1)
    for j=1:size(mapInflated,2)
        if mapInflated(i,j)>0
            % plot a filled translucent square
            x = [i-0.5 i-0.5 i+0.5 i+0.5];
            y = [j-0.5 j+0.5 j+0.5 j-0.5];
            h = fill(x,y,'k','LineStyle','none');
            set(h,'facealpha',.5)
        end
    end
end

for k=1:size(plan,1)
    % plot a filled square
    i = plan(k,1);
    j = plan(k,2);
    x = [i-0.5 i-0.5 i+0.5 i+0.5];
    y = [j-0.5 j+0.5 j+0.5 j-0.5];
    fill(x,y,'g','LineStyle','none');
end

% plot(plan(:,1),plan(:,2),'r','Linewidth');
axis equal;
axis([0 Nw 0 Nh]);
grid on;
titlestr = {'Known obstacle grid map','and global plan in grid coordinates'};
title(titlestr);