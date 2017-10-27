clc; clear;

global dcps;

% general parameters
dt        = 0.001;      % time step of training trajectory
start     = 0;          % DMP start-position
goal      = 1;          % DMP end-position
tau       = 1;          % DMP time scaling constant
n_rfs     = 6;         % # of basis functions to use/DMP
ID        = 1;          % matlab flag to use dmp library

% initialize DMP
dcp('clear',ID);
dcp('init',ID,n_rfs,'t^3_dmp',1);

% initialize some variables for plotting
T=zeros(floor(tau/dt+1),3);           % trajectory (user-specified)
Y=T;                                    % dmp-computed trajectory
PSI=zeros(floor(tau/dt+1),n_rfs);     % storage for gaussians
W=PSI;                                  % gaussian weights

% generating y = t^3 trajectory
t = 0:dt/(tau):1;
y = t.^3;
T(:,1) = y;
T(2:end,2) = diff(T(:,1), 1)/(dt^1);
T(3:end,3) = diff(T(:,1), 2)/(dt^2);

% computing DMP
dcp('Batch_Fit',ID,tau,dt,T(:,1),T(:,2),T(:,3));

% running DMP to goal
dcp('reset_state',ID,start);
dcp('set_goal',ID,goal,1); %the "1" resets y0 and x
for i=0:tau/dt,
  [y,yd,ydd]=dcp('run',ID,tau,dt);
  Y(i+1,:)   = [y yd ydd];
  PSI(i+1,:) = dcps(ID).psi';
  W(i+1,:)   = dcps(ID).w';
end;
  
% plotting
time = (0:dt:tau)';
figure(1);
clf;

% plot position, velocity, acceleration vs. target
subplot(2,3,1);
plot(time,[Y(:,1) T(:,1)]);
title('y');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(2,3,2);
plot(time,[Y(:,2) T(:,2)]);
title('yd');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(2,3,3);
plot(time,[Y(:,3) T(:,3)]);
title('ydd');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

% plot PSI and weight info
subplot(2,3,4);
plot(time,PSI);
title('Weighting Kernels');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(2,3,5);
plot(time,W);
title('Linear Model Weights over Time');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(2,3,6);
plot(W(end,:));
title('Weights');
xlabel(sprintf('tau=%f',tau));

drawnow;
