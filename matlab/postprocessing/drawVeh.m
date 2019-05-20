function drawVeh(p,phi,L,arc,u)
% Function that draws the vehicle in world coordinate frame.

% vehicle inside color
vehicleInsideColor  = [170 170 200]/255;
th                  = 0:pi/50:2*pi;
x_circle            = p(1)+0.5*L*cos(th);
y_circle            = p(2)+0.5*L*sin(th);
fill(x_circle, y_circle,vehicleInsideColor);
% vehicle outline
plot(p(1)+0.5*L*cos(arc), p(2)+0.5*L*sin(arc),'k',...
'LineWidth',1.5);
% vehicle inside line connecting wheels to indicate orientation
plot([p(1)+0.5*L*cos(phi), p(1)-0.5*L*cos(phi)],...
    [p(2)+0.5*L*sin(phi), p(2)-0.5*L*sin(phi)],'k','LineWidth',1)
% vehicle wheel velocity arrows
quiver(p(1)+0.5*L*cos(phi),p(2)+0.5*L*sin(phi),...
    -u(1)*sin(phi),u(1)*cos(phi),'k-','LineWidth',1.4);
quiver(p(1)-0.5*L*cos(phi),p(2)-0.5*L*sin(phi),...
    -u(2)*sin(phi),u(2)*cos(phi),'k-','LineWidth',1.4);