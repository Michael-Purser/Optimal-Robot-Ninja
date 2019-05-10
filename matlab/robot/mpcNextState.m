function MPC = mpcNextState(MPC,veh,nSelector,stateUpdateSelector)
% Function that calculates how many states to move the vehicle between
% successive MPC iterations.
% Different strategies are implemented for comparison.

switch nSelector
    
    % Strategy 1: Based on distance to target (old)
    case 1
        n_original = MPC.nav.opt.horizon;
        n_min      = ceil(veh.Optim.n/10);
        n_max      = ceil(veh.Optim.n/10);
        n_new = ceil(n_original*(1-norm([MPC.nav.currentState(1:2);1]-[MPC.nav.globalGoal(1:2);1])/norm([MPC.nav.globalStart(1:2);1]-[MPC.nav.globalGoal(1:2);1])));
        if n_new<n_min
            n_new = n_min;
        elseif n_new>n_max
            n_new = n_max;
        end
    
    % Strategy 2: based on actuation frequency
    case 2
        f_min   = veh.motors.fmax; % Hz
        if MPC.nav.opt.sol.T == 0
            T_n = MPC.nav.opt.init.T;
        else
            T_n = MPC.nav.opt.sol.T(end);
        end
        n = MPC.nav.opt.horizon;
        n_new = ceil(n/(T_n*f_min));
        if n_new>n
            n_new = n;
        end
        
    % Strategy 3: same as 2, but with exact change (instead of ceil)
    case 3
        f_min   = veh.motors.fmax; % Hz
        if MPC.nav.opt.sol.T == 0
            T_n = MPC.nav.opt.init.T;
        else
            T_n = MPC.nav.opt.sol.T(end);
        end
        n = MPC.nav.opt.horizon;
        n_new = 1+n/(T_n*f_min); % +1 because of matlab indexing!!
        if n_new>n
            n_new = n;
        end
end

% update position, orientation and control signals:
curPos  = MPC.nav.currentState;
solX    = MPC.nav.opt.sol.x;
solU    = MPC.nav.opt.sol.u;
T_n     = MPC.nav.opt.sol.T(end);
n       = MPC.nav.opt.horizon;
noiseA  = veh.motors.noiseamp;
L       = veh.geometry.wheelBase;
Umax    = veh.dynamics.velLimits(2);

switch stateUpdateSelector
    
    % Strategy 1: exact update
    case 1
        if nSelector == 1 || nSelector == 2
            newPos = homTrans(curPos(3),[curPos(1:2);1])*[solX(1:2,n_new+1);1];
            newOri = curPos(3)-solX(3,n_new+1);
            newU   = solU(:,n_new+1);
        elseif nSelector == 3
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
    
    % Strategy 2: update by integration of noisy velocity signals
    case 2
        % this strategy updates the state by integrating the solution,
        % after adding white noise to simulate real-world effects
        
        % add white noise to velocity signals
        Unoisy = zeros(size(solU));
        for i=1:size(solU,2)
            noiseR = (rand(1)-0.5)*noiseA*Umax;
            noiseL = (rand(1)-0.5)*noiseA*Umax;
            Unoisy(1,i) = solU(1,i)+noiseR;
            Unoisy(2,i) = solU(2,i)+noiseL;
        end
        
        % get omega signal using dynamic equations
        omega = (2/L)*(Unoisy(1,:)-Unoisy(2,:));
        
        % get theta value of state (in local frame) by integrating omega 
        % signal (simple Euler)
        % NOTE: the - in the loop is because of the 'awkward' definition of
        % the robot's orientation phi, which is positive in the opposite
        % direction than omega
        theta = zeros(1,size(solU,2));
        for i=2:n
            theta(i) = theta(i-1)-(T_n/n)*omega(i-1);
        end
        
        % get vx, vy and omega values using the dynamic equations
        vx = (Unoisy(1,:)+Unoisy(2,:))/2 .* sin(theta);
        vy = (Unoisy(1,:)+Unoisy(2,:))/2 .* cos(theta);
        
        % get x, y values of state (in local frame) by integrating velocity 
        % signals (simple Euler)
        x = zeros(1,size(solU,2));
        y = zeros(1,size(solU,2));
        for i=2:n
            x(i) = x(i-1)+(T_n/n)*vx(i-1);
            y(i) = y(i-1)+(T_n/n)*vy(i-1);
        end
        
        % get state values corresponding to n_new (in local frame)
        if nSelector == 1 || nSelector == 2
            xnew        = x(n_new+1);
            ynew        = y(n_new+1);
            thetanew    = theta(n_new+1);
            newU        = Unoisy(:,n_new+1);
        elseif nSelector == 3
            m      = floor(n_new);
            f      = n_new-m;
            xnew        = x(m) + f*(x(m+1)-x(m));
            ynew        = y(m) + f*(y(m+1)-y(m));
            thetanew    = theta(m) + f*(theta(m+1)-theta(m));
            if m<n
                newU = Unoisy(:,m) + f*(Unoisy(:,m+1)-Unoisy(:,m));
            else
                newU = Unoisy(:,m);
            end
        end
        
        % transform values to global frame:
        newOri      = curPos(3) - thetanew; % again due to awkward angle definition
        newPos      = homTrans(curPos(3),[curPos(1:2);1])*[xnew;ynew;1];
        
        % plots for debug:
%         figure;
%         subplot(121);
%         plot(solU(1,:)); hold on;
%         plot(Unoisy(1,:));
%         legend('solution ur','noisy ur');
%         title('Solution and noisy ur');
%         subplot(122);
%         plot(solU(2,:)); hold on;
%         plot(Unoisy(2,:));
%         legend('solution ul','noisy ul');
%         title('Solution and noisy ul');
%         
%         figure;
%         subplot(121);
%         plot(omega);
%         title('Noisy \omega');
%         subplot(122);
%         plot(solX(3,:)); hold on;
%         plot(theta);
%         legend('exact \theta','integrated \theta');
%         title('Exact and integrated \theta');
%         
%         figure;
%         plot(vx); hold on;
%         plot(vy);
%         legend('vx','vy');
%         title('Noisy vx and vy');
%         
%         figure;
%         subplot(121);
%         plot(x); hold on;
%         plot(solX(1,:));
%         legend('exact x','solution x');
%         title('Exact and integrated x');
%         subplot(122);
%         plot(y); hold on;
%         plot(solX(2,:));
%         legend('exact y','solution y');
%         title('Exact and integrated y');
%         
%         figure;
%         plot(diff(x)); hold on;
%         plot(diff(solX(1,:)));
%         legend('integrated','exact');
%         title('diff of exact and of integrated x');
        
end
        

% add to situation struct:
MPC.nav.m            = n_new;
MPC.nav.currentState = [newPos(1:2);newOri];
MPC.nav.currentVelocity  = newU;

end