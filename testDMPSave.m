clc; clear; close;

% DMP training parameters
name = 'test.xml';
n_rfs = 256;
dt  = 0.001;

%Start and end parameters
start = 0;
goal = 5;

% generating y = t^3 trajectory
t = 0:dt:1;
y = t.^3;
T(:,1) = y;
T(2:end,2) = diff(T(:,1), 1)/(dt^1);
T(3:end,3) = diff(T(:,1), 2)/(dt^2);

% DMP running parameters
tau = 1.0;

% YOUR TEST HERE %
train_dmp(name, n_rfs, T, dt);
% dcp('reset_state',ID,start);
% dcp('set_goal',ID,goal,1); %the "1" resets y0 and x
myRunner = DMP_Runner(name);

%Set start point
myRunner.setStart(start);

%Set goal point
myRunner.setGoal(goal,1);

for i=0:tau/dt
  Y(i+1,:) = myRunner.step(tau, dt);
  %Dynamic change in goal at first quaarter
  %if i>0.25*(tau/dt)
  %    myRunner.setGoal(5,0);
  %end
end

% plot position vs. target
time = (0:dt:tau)';
plot((0:dt:tau)',[Y(:,1) T(:,1)]);
aa=axis; axis([min(time) max(time) aa(3:4)]);
% YOUR TEST HERE %

% TODO's:
% Clean up codebase; write good demo code