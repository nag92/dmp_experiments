clc; clear;
%% Train DMP
% DMP training parameters
n_rfs = 256;        % how many basis functions to train with
dt  = 0.001;        % integration timestep

% create DMPs (save to .xml)
[xT, yT, zT] = makeCardioid(1, dt);
train_dmp('3-dof_x-dim.xml', n_rfs, xT, dt);
train_dmp('3-dof_y-dim.xml', n_rfs, yT, dt);
train_dmp('3-dof_z-dim.xml', n_rfs, zT, dt);

%% Use trained DMP
% DMP running parameters
tau = 1.0;          % go relatively faster/slower than trained DMP
start = [0,0,0];    % set desired start-points of trajctory
goal = [0,0,0];     % set desired end-points of trajectory

% load in DMPs
xRunner = DMP_Runner('3-dof_x-dim.xml', start(1), goal(3));
yRunner = DMP_Runner('3-dof_y-dim.xml', start(2), goal(3));
zRunner = DMP_Runner('3-dof_z-dim.xml', start(3), goal(3));

% Generate trajectories, (X, Y, Z), using DMP
%simulation setup
h = figure(1); hold on; view(3); grid on
arm_plot  = plot3([0],[0],[0], 'Marker', 'o', 'LineStyle','-');
traj_plot = plot3([0],[0],[0], 'LineStyle','-');
axis([-5 5 -5 5 -5 5]);
history = [];

for i=0:tau/dt
  % time step
  X(i+1,:) = xRunner.step(tau, dt);
  Y(i+1,:) = yRunner.step(tau, dt);
  Z(i+1,:) = zRunner.step(tau, dt);
  
  % update simulation
  arm_pts = getArmPts(X(i+1,1), Y(i+1,1), Z(i+1,1));
  history(i+1,:) = arm_pts(4,:);
  
  set(arm_plot,'XData', arm_pts(:,1), 'YData', arm_pts(:,2), 'ZData', arm_pts(:,3));
  set(traj_plot,'XData', history(:,1), 'YData', history(:,2), 'ZData', history(:,3));
  pause(dt);
end

%% Analyze Results
figure(2); clf; hold on; view(3);
plot3(X(:,1), Y(:,1), Z(:,1));
plot3(xT(:,1), yT(:,1), zT(:,1));
title('training (pos) trajectory vs. generated (pos) trajectory');

% compare individual DMP's
time = (0:dt:tau)';
dir_num = 1; % pos = 1, vel = 2, accl = 3
figure(3); clf;

subplot(3,1,1);
plot(time,[X(:,dir_num) xT(:,dir_num)]);
title(sprintf('%d derivative X-trajectory',dir_num-1));
aa=axis; axis([min(time) max(time) aa(3:4)]);

subplot(3,1,2);
plot(time,[Y(:,dir_num) yT(:,dir_num)]);
title(sprintf('%d derivative Y-trajectory',dir_num-1));
aa=axis; axis([min(time) max(time) aa(3:4)]);

subplot(3,1,3);
plot(time,[Z(:,dir_num) zT(:,dir_num)]);
title(sprintf('%d derivative Z-trajectory',dir_num-1));
aa=axis; axis([min(time) max(time) aa(3:4)]);

drawnow;
