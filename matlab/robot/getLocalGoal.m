function MPC = getLocalGoal(MPC)

x       = MPC.nav.currentState;
g       = MPC.nav.globalGoal(1:2);

% transform goal position from global coordinates to local robot
% coordinates
T = homTrans(x(3),[x(1:2);1]);
G = T\[g;1];
MPC.nav.opt.goal = [G(1:2);MPC.nav.globalGoal(3)+x(3)];

end