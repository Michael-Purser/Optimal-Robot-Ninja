function env = sortObstacles(MPC,env)
    % Function that sorts the obstacles into 'measured' and 'mapped' based
    % on the selected mode:
    
    if MPC.nav.mapObstacles
        for i=1:size(env.obst,2)
            mode = env.obst{i}.mode;
            if strcmp(mode,'known')
                env.mapped{end+1} = env.obst{i};
            else
                env.measured{end+1} = env.obst{i};
            end
        end
    else
        env.measured = env.obst;
    end

end