clc; clear;
global dcps;

% generaler parameters
outfile = '32.gif';
saveAnimation = 0;

% general parameters
dt        = 0.001;      % time step of training trajectory
start     = [0,0,0];    % DMP start-position [x,y,z]
goal      = [0,0,0];    % DMP end-position   [z,y,z]
tau       = 2.0;        % DMP time scaling constant
n_rfs     = 100;        % # of basis functions to use/DMP

% initialize a DMP for each of the X, Y, and Z dimensions
Xid = 1; dcp('clear',Xid); dcp('init',Xid,n_rfs,'X-dim_dmp',1);
Yid = 2; dcp('clear',Yid); dcp('init',Yid,n_rfs,'Y-dim_dmp',1);
Zid = 3; dcp('clear',Zid); dcp('init',Zid,n_rfs,'Z-dim_dmp',1);

% initialize some variables for plotting
[xT, yT, zT] = makeCardioid(tau, dt); % training trajectories
X = zeros(floor(tau/dt+1),3);       % dmp-computed x-trajectory
Y = zeros(floor(tau/dt+1),3);       % dmp-computed y-trajectory
Z = zeros(floor(tau/dt+1),3);       % dmp-computed z-trajectory

% compute DMP for each of the X, Y, and Z dimensions
dcp('Batch_Fit',Xid,tau,dt,xT(:,1),xT(:,2),xT(:,3));
dcp('Batch_Fit',Yid,tau,dt,yT(:,1),yT(:,2),yT(:,3));
dcp('Batch_Fit',Zid,tau,dt,zT(:,1),zT(:,2),zT(:,3));

% run DMPs from start to goal
dcp('reset_state',Xid,start(1)); dcp('set_goal',Xid,goal(1),1);
dcp('reset_state',Yid,start(2)); dcp('set_goal',Yid,goal(2),1);
dcp('reset_state',Zid,start(3)); dcp('set_goal',Zid,goal(3),1);

%simulation setup
h = figure(3); hold on; view(3); grid on
arm_plot  = plot3([0],[0],[0], 'Marker', 'o', 'LineStyle','-');
traj_plot = plot3([0],[0],[0], 'LineStyle','-');
axis([-5 5 -5 5 -5 5]);
history = [];

tic
for i=0:tau/dt
  % change goal if more than halfway
     if(i > 0.5*tau/dt) %change goal
       goal = [1,1,1];
       dcp('set_goal',Xid,goal(1),0);
       dcp('set_goal',Yid,goal(2),0);
       dcp('set_goal',Zid,goal(3),0);
     end
  
  % next task-space point computations
  X(i+1,:)   = dcp('run',Xid,tau,dt);
  Y(i+1,:)   = dcp('run',Yid,tau,dt);
  Z(i+1,:)   = dcp('run',Zid,tau,dt);
  
  % update drawing
  arm_pts = getArmPts(X(i+1,1), Y(i+1,1), Z(i+1,1));
  history(i+1,:) = arm_pts(4,:);
  
  set(arm_plot,'XData', arm_pts(:,1), 'YData', arm_pts(:,2), 'ZData', arm_pts(:,3));
  set(traj_plot,'XData', history(:,1), 'YData', history(:,2), 'ZData', history(:,3));
  pause(dt);
  
  if saveAnimation == 1
      frame = getframe(h);
      im = frame.cdata;
      [imind,cm] = rgb2ind(im,256); 
      if i == 0 
          imwrite(imind,cm,outfile,'gif','DelayTime',dt,'loopcount',inf); 
      else 
          imwrite(imind,cm,outfile,'gif','DelayTime',dt,'writemode','append'); 
      end
  end
end
toc
  
% plot 3-space trajectory
figure(1); clf; hold on; view(3);
plot3(X(:,1), Y(:,1), Z(:,1));
plot3(xT(:,1), yT(:,1), zT(:,1));
title('training (pos) trajectory vs. generated (pos) trajectory');

% compare individual DMP's
time = (0:dt:tau)';
dir_num = 1; % pos = 1, vel = 2, accl = 3
figure(2); clf;

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

function [xT, yT, zT] = makeCardioid(tau, dt)
% useful anonymous functions
rotx  = @(th) [1 0 0; 0 cos(th) -sin(th); 0 sin(th) cos(th)];
roty  = @(th) [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)];
rotz  = @(th) [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];

xT=zeros(floor(tau/dt+1),3);          % [pos; vel; accl]
yT=zeros(floor(tau/dt+1),3);          % [pos; vel; accl]
zT=zeros(floor(tau/dt+1),3);          % [pos; vel; accl]

% generating rotated cardioid trajectory
t = (0:dt/(tau):1)'*2*pi;               % parametric parameter for traj.

T = [2*(1-cos(t)).*cos(t), ...          % [xPos; yPos; zPos]
     2*(1-cos(t)).*sin(t), ...
     zeros(floor(tau/dt+1),1)];

T = T*rotx(pi/4)*roty(pi/4)*rotz(pi/4);

% extract trajectory into useful data shapes
xT(:,1) = T(:,1);
xT(2:end,2) = diff(xT(:,1), 1)/(dt^1);
xT(3:end,3) = diff(xT(:,1), 2)/(dt^2);

yT(:,1) = T(:,2);
yT(2:end,2) = diff(yT(:,1), 1)/(dt^1);
yT(3:end,3) = diff(yT(:,1), 2)/(dt^2);

zT(:,1) = T(:,3);
zT(2:end,2) = diff(zT(:,1), 1)/(dt^1);
zT(3:end,3) = diff(zT(:,1), 2)/(dt^2);
end
