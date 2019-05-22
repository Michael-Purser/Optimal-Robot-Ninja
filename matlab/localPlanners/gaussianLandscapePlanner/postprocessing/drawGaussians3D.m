function drawGaussians3D(gmap,xVec,yVec,Ghat)
% Function that draws a 3D representation of the gaussian landscape

mesh(xVec,yVec,gmap'); axis off;
contour3(xVec,yVec,gmap',[Ghat,Ghat],'r','Linewidth',4);