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