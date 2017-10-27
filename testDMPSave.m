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
if(i == 50)
   'hi' 
end



  [y, yd, ydd] = myRunner.step(tau, dt);
  Y(i+1,:) = [y; yd; ydd];
end
% YOUR TEST HERE %

% plot position vs. target
time = (0:dt:tau)';
plot((0:dt:tau)',[Y(:,1) T(:,1)]);
aa=axis; axis([min(time) max(time) aa(3:4)]);