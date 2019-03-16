function sit = mpcNextState(sit,veh,selector)
% Function that calculates how many states to move the vehicle between
% successive MPC iterations.
% Different strategies are implemented for comparison.

switch selector
    
    % Strategy 1: Based on distance to target (old)
    case 1
        n_original = veh.Optim.n;
        n_min      = ceil(veh.Optim.n/10);
        n_max      = ceil(veh.Optim.n/10);
        n_new = ceil(n_original*(1-norm([sit.states{end}(1:2);1]-[sit.goalState(1:2);1])/norm([sit.startState(1:2);1]-[sit.goalState(1:2);1])));
        if n_new<n_min
            n_new = n_min;
        elseif n_new>n_max
            n_new = n_max;
        end
    
    % Strategy 2: based on actuation frequency
    case 2
        f_min   = veh.actuatorfMin; % Hz
        if isempty(sit.Sol.T)
            T_n = sit.Init.T{end};
        else
            T_n = sit.Sol.T{end};
        end
        n = veh.Optim.n;
        n_new = ceil(n/(T_n*f_min));
        if n_new>n
            n_new = n;
        end
        
    % Strategy 3: same as 2, but with exact change (instead of ceil)
    case 3
        f_min   = veh.actuatorfMin; % Hz
        if isempty(sit.Sol.T)
            T_n = sit.Init.T{end};
        else
            T_n = sit.Sol.T{end}(end);
        end
        n = veh.Optim.n;
        n_new = 1+n/(T_n*f_min); % +1 because of matlab indexing!!
        if n_new>n
            n_new = n;
        end
end

% update position, orientation and control signals:
curPos = sit.states{end};
solX   = sit.Sol.X{end};
solU   = sit.Sol.U{end};
if selector == 1 || selector == 2
    newPos = homTrans(curPos(3),[curPos(1:2);1])*[solX(1:2,n_new+1);1];
    newOri = curPos(3)-solX(3,n_new+1);
    newU   = solU(:,n_new+1);
elseif selector == 3
    m      = floor(n_new);
    f      = n_new-m;
    locPos = solX(:,m)+f*(solX(:,m+1)-solX(:,m));
    newPos = homTrans(curPos(3),[curPos(1:2);1])*[locPos(1:2);1];
    newOri = curPos(3)-locPos(3);
    if m<n
        newU = solU(:,m)+f*(solU(:,m+1)-solU(:,m));
    else
        newU = solU(:,m);
    end
end

% add to situation struct:
sit.nNew{end+1} = n_new;
sit.states{end+1} = [newPos(1:2);newOri];
sit.controls{end+1} = newU;

end