clc; clear; close;

% DMP training parameters
name = 'test.xml';
n_rfs = 6;
dt  = 0.001;

% generating y = t^3 trajectory
t = 0:dt:1;
y = t.^3;
T(:,1) = y;
T(2:end,2) = diff(T(:,1), 1)/(dt^1);
T(3:end,3) = diff(T(:,1), 2)/(dt^2);

% DMP running parameters
tau = 1.0;

% YOUR TEST HERE %
[w, D, c] = train_dmp(name, n_rfs, T, dt);
% dcp('reset_state',ID,start);
% dcp('set_goal',ID,goal,1); %the "1" resets y0 and x

myRunner = DMP_Runner(w, D, c);
for i=0:tau/dt
  Y(i+1,:) = myRunner.step(tau, dt);
end

% plot position vs. target
time = (0:dt:tau)';
plot((0:dt:tau)',[Y(:,1) T(:,1)]);
aa=axis; axis([min(time) max(time) aa(3:4)]);
% YOUR TEST HERE %

% TODO's:
% figure out how the set the start position
% read/write to/from XML
% add back in functional scaling if you want it
% add back in 0th order canonical function if you want it
% invert 1/tau if possible