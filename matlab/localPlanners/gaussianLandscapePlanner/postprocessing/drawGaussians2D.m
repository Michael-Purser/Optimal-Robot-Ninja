function drawGaussians2D(gmap,xVec,yVec,Ghat,levels)
% Function that draws a 2D contour map of the gaussian landscape

contour(xVec,yVec,gmap',levels);
contour(xVec,yVec,gmap',[Ghat Ghat],'LineWidth',2,'LineColor','r');