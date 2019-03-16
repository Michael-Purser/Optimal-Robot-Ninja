function mpc_makeMov(sit,veh,env,sitStr,count,selector)
% Function that makes the videos for the tested MPC run.

fprintf('Starting making movie \n');

switch selector
    
    % Different trajectories with gaussians and projected paths
    case 1
        for k = 1:count
            mpc_plotSol(sit,veh,env,k,0);
            drawnow;
            set(gcf,'Position',[1 1 450*2 570*2]);
            F(k) = getframe(gcf);
            close(gcf);
            fprintf('making frame %i out of %i \n',k,size(sit.nNew,2)-1)
        end
        name = [sitStr,'_simple','.avi'];
        v = VideoWriter(name);
        v.Quality = 100;
        v.FrameRate = 1;
        open(v);
        writeVideo(v,F);
        close(v);
        
    % Like case 1, but add global path intialization
    case 2
        for k = 1:count
            mpc_plotSol(sit,veh,env,k,1);
            drawnow;
            set(gcf,'Position',[1 1 450*2 570*2]);
            F(k) = getframe(gcf);
            close(gcf);
            fprintf('making frame %i out of %i \n',k,size(sit.nNew,2)-1)
        end
        name = [sitStr,'_extended','.avi'];
        v = VideoWriter(name);
        v.Quality = 100;
        v.FrameRate = 1;
        open(v);
        writeVideo(v,F);
        close(v);
        
end    
save('F.mat','F');

fprintf('done \n')
        
end