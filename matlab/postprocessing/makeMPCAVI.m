function makeMPCAVI(log,veh,env)

totalFrames         = size(log.states,2)-1;
framesPerPart       = 10;
numberOfParts       = ceil(totalFrames/framesPerPart);
remainder           = rem(totalFrames,framesPerPart);

% info displays:
fprintf('\n');
fprintf('Starting video creation...\n');
fprintf('Number of part files: \t %i \n', numberOfParts);
fprintf('Frames per part file: \n');
for m=1:numberOfParts
    mframes = framesPerPart;
    if m==numberOfParts && remainder>0
        mframes = remainder;
    end
    fprintf('\t Part %i: \t %i \n', m, mframes);
end
fprintf('\n');

for m=1:numberOfParts
    
    % clear F struct
    clear F;
    
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
        
        k = (m-1)*framesPerPart + i;
        
        plotMPCState(log,veh,env,k,1); % '1' is extra input to turn off command line displays
        
        drawnow;
        pause(0.5);
        set(gcf,'Position',[1000 1 800 800]);
        F(i) = getframe(gcf);
        close(gcf);
        
        % completion logger
        perc = floor(100*i/n);
        if rem(perc,10)==0 && perc>prev_perc
            fprintf('\t %i%% \t \t %i/%i \n',perc,k,totalFrames);
            prev_perc = perc;
        end
    end
    
    fprintf('Saving... \n');
    save(matfile,'F');
    
    fprintf('Writing avi... \n');
    name = movfile;
    v = VideoWriter(name);
    v.Quality = 90;
    v.FrameRate = 5;
    open(v);
    writeVideo(v,F);
    close(v);
    fprintf('Finished writing avi \n');
    fprintf('\n');

end