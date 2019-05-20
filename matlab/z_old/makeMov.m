function makeMov(sit,veh,env,name,FRfactor,qfactor)
% Function that makes the videos for the single optimization.
% Inputs:
%       sit, veh, env       structs resulting from the optimization routine
%       name                name of the movie
%       FRfactor            acceleration factor for the movie framerate
%                           (for most accurate representation of reality 
%                           set to 1.0);
%       qfactor             quality factor for the movie; the higher this
%                           number, the more accurate the gaussian contours
%                           will be. Cannot be set below 1.0 .
% Output:
%       Saves the created movie in the local directory as 'name.avi'

for k = 1:veh.Optim.n
    fprintf('making frame %i of %i \n',k,veh.Optim.n);
    makeMovFrame(sit,veh,env,k,qfactor);
    drawnow;
    set(gcf,'Position',[1 1 450*2 570*2]);
    F(k) = getframe(gcf);
    close(gcf);
end

% adapt framerate:
FR = FRfactor*floor(veh.Optim.n/sit.Sol.T{end});
if FR<1
    fprintf('FR too small; setting to 1 \n');
end

% add 1 second of frames at start and at end of movie:
for j = 1:FR
    F = [F(1) F];
    F(end+1) = F(end);
end

% compile movie:
fprintf('compiling movie \n');
name = [name,'.avi'];
v = VideoWriter(name);
v.Quality = 100;
v.FrameRate = FR;
open(v);
writeVideo(v,F);
close(v);
fprintf('done \n');

end