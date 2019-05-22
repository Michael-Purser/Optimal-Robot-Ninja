function plotDynamics(log,it)
% Plot planned platform dynamics along MPC iteration 'it'.

fprintf('Plotting vehicle dynamics along solution %i \n',it);

% get data
U           = log.localPlanners{it}.sol.u;
T           = log.localPlanners{it}.sol.T;
horizon     = log.localPlanners{it}.params.horizon;
velLimits   = log.localPlanners{it}.params.dynLimits.vel;
accLimits   = log.localPlanners{it}.params.dynLimits.acc;
jerkLimits  = log.localPlanners{it}.params.dynLimits.jerk;

% reconstruct dynamic properties
[A,J,dT] = getAccelerationAndJerk(U,T,horizon);

% plot velocities and accelerations in wheels:
t_vec = linspace(0,T(end),horizon+1);

slack = 0.1;
[vRightUpper,vRightLower] = getAxisLimits(max(max(U(1,:)),velLimits(2)),min(min(U(1,:)),velLimits(1)),slack);
[vLeftUpper,vLeftLower]   = getAxisLimits(max(max(U(2,:)),velLimits(2)),min(min(U(2,:)),velLimits(1)),slack);
[aRightUpper,aRightLower] = getAxisLimits(max(max(A(1,:)),accLimits(2)),min(min(A(1,:)),accLimits(1)),slack);
[aLeftUpper,aLeftLower]   = getAxisLimits(max(max(A(2,:)),accLimits(2)),min(min(A(2,:)),accLimits(1)),slack);
[jRightUpper,jRightLower] = getAxisLimits(max(max(J(1,:)),jerkLimits(2)),min(min(J(1,:)),jerkLimits(1)),slack);
[jLeftUpper,jLeftLower]   = getAxisLimits(max(max(J(2,:)),jerkLimits(2)),min(min(J(2,:)),jerkLimits(1)),slack);

figure;

subplot(3,2,1);
hold all;
plot(t_vec(1:end-1),U(1,:)); xlabel('time [s]'); ylabel('v_{right}(t) [m/s]'); title('v_{right}(t)');
plot(t_vec(1:end-1),ones(1,size(U,2))*velLimits(1),'r--');
plot(t_vec(1:end-1),ones(1,size(U,2))*velLimits(2),'r--');
axis([0 t_vec(end) vRightLower vRightUpper]);

subplot(3,2,2);
hold all;
plot(t_vec(1:end-1),U(2,:)); xlabel('time [s]'); ylabel('v_{left}(t) [m/s]'); title('v_{left}(t)');
plot(t_vec(1:end-1),ones(1,size(U,2))*velLimits(1),'r--');
plot(t_vec(1:end-1),ones(1,size(U,2))*velLimits(2),'r--');
axis([0 t_vec(end) vLeftLower vLeftUpper]);

subplot(3,2,3);
hold all;
plot(t_vec(1:end-2)+dT/2,A(1,:)); xlabel('time [s]'); ylabel('a_{right}(t) [m/s^2]'); title('a_{right}(t)');
plot(t_vec(1:end-2),ones(1,size(A,2))*accLimits(1),'r--');
plot(t_vec(1:end-2),ones(1,size(A,2))*accLimits(2),'r--');
axis([0 t_vec(end) aRightLower aRightUpper]);

subplot(3,2,4);
hold all;
plot(t_vec(1:end-2)+dT/2,A(2,:)); xlabel('time [s]'); ylabel('a_{left}(t) [m/s^2]'); title('a_{left}(t)');
plot(t_vec(1:end-2),ones(1,size(A,2))*accLimits(1),'r--');
plot(t_vec(1:end-2),ones(1,size(A,2))*accLimits(2),'r--');
axis([0 t_vec(end) aLeftLower aLeftUpper]);

subplot(3,2,5);
hold all;
plot(t_vec(1:end-3)+dT,J(1,:)); xlabel('time [s]'); ylabel('j_{right}(t) [m/s^3]'); title('j_{right}(t)');
plot(t_vec(1:end-3),ones(1,size(J,2))*jerkLimits(1),'r--');
plot(t_vec(1:end-3),ones(1,size(J,2))*jerkLimits(2),'r--');
axis([0 t_vec(end) jRightLower jRightUpper]);

subplot(3,2,6);
hold all;
plot(t_vec(1:end-3)+dT,J(2,:)); xlabel('time [s]'); ylabel('j_{left}(t) [m/s^3]'); title('j_{left}(t)');
plot(t_vec(1:end-3),ones(1,size(J,2))*jerkLimits(1),'r--');
plot(t_vec(1:end-3),ones(1,size(J,2))*jerkLimits(2),'r--');
axis([0 t_vec(end) jLeftLower jLeftUpper]);

end