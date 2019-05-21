function MPC = actuate(MPC,veh,localPlanner,selector)

% update position, orientation and control signals:
curPos  = MPC.currentState;
n_new   = MPC.m;
solX    = localPlanner.sol.x;
solU    = localPlanner.sol.u;
T_n     = localPlanner.sol.T(end);
n       = localPlanner.params.horizon;
noiseA  = veh.motors.noiseamp;
L       = veh.geometry.wheelBase;
Umax    = veh.dynamics.velLimits(2);

switch selector
    
    % Strategy 1: exact update
    case 1
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
        theta = zeros(1,size(solX,2));
        for i=2:n
            theta(i) = theta(i-1)-(T_n/n)*omega(i-1);
        end
        
        % get vx, vy and omega values using the dynamic equations
        vx = (Unoisy(1,:)+Unoisy(2,:))/2 .* sin(theta(1:end-1)); % because states 1 longer than velocities
        vy = (Unoisy(1,:)+Unoisy(2,:))/2 .* cos(theta(1:end-1));
        
        % get x, y values of state (in local frame) by integrating velocity 
        % signals (simple Euler)
        x = zeros(1,size(solX,2));
        y = zeros(1,size(solX,2));
        for i=2:n
            x(i) = x(i-1)+(T_n/n)*vx(i-1);
            y(i) = y(i-1)+(T_n/n)*vy(i-1);
        end
        
        % get state values corresponding to n_new (in local frame)
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
        
        % transform values to global frame:
        newOri      = curPos(3) - thetanew; % again due to awkward angle definition
        newPos      = homTrans(curPos(3),[curPos(1:2);1])*[xnew;ynew;1];
end

MPC.currentState = [newPos(1:2);newOri];
MPC.currentVelocity  = newU;