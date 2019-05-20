function mpc_makeAVI(MPC,veh,env,N)

totalFrames         = size(MPC.log.states,2)-1;
framesPerPart       = 100;
numberOfParts       = ceil(totalFrames/framesPerPart);
remainder           = rem(totalFrames,framesPerPart);

for m=1:numberOfParts
    
    % iteration log
    fprintf('Making file %i/%i \n',m,numberOfParts);
    fprintf('\t Completion \t Total \n');
    
    % make movie part save file name
    filestr  = ['part_',num2str(m)];
    pathname = fileparts('./figs/');
    matfile  = fullfile(pathname, [filestr '.mat']);
    movfile  = fullfile(pathname, [filestr '.avi']);

    % get nb frames in this part
    n = framesPerPart;
    if m==numberOfParts && remainder>0
        n = remainder;
    end
    
    % initialise counter for percentage completion logger in next loop
    prev_perc = 0;
    
    for i=1:n
        
        k = (m-1)*n + i;
        
        % completion logger
        perc = floor(100*i/n);
        if rem(perc,10)==0 && perc>prev_perc
            fprintf('\t %i%% \t \t %i/%i \n',perc,k,n);
            prev_perc = perc;
        end
        
        mpc_plotSol(MPC,veh,env,k,N,false); 
        
        drawnow;
        pause(0.5);
        set(gcf,'Position',[1000 1 800 800]);
        F(i) = getframe(gcf);
        close(gcf);
    end
    
    fprintf('\t Saving... \n');
    save(matfile,'F');
    
    fprintf('\t Writing avi... \n');
    name = movfile;
    v = VideoWriter(name);
    v.Quality = 90;
    v.FrameRate = 5;
    open(v);
    writeVideo(v,F);
    close(v);
    fprintf('\t Finished writing avi \n');

end