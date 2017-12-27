clc; clear; close;
%% Train DMP
% DMP training parameters
name = 'test.xml';  % file to save DMP to
n_rfs = 256;        % how many basis functions to train with
dt  = 0.001;        % integration timestep

% create DMP (save to .xml)
T = y_cubed(dt);
train_dmp(name, n_rfs, T, dt);

%% Use trained DMP
% DMP running parameters
tau = 2.0;          % go relatively faster/slower than trained DMP
start = 0;          % set desired start-point of trajctory
goal = 1;           % set desired end-point of trajectory

% load in DMP
myRunner = DMP_Runner(name, start, goal);

% Generate trajectory, Y, using DMP
for i=0:tau/dt
  Y(i+1,:) = myRunner.step(tau, dt);
end

%% Analyze Results
time = (0:dt:tau)';
plot((0:dt:tau)',[Y(:,1) T(:,1)]);
aa=axis; axis([min(time) max(time) aa(3:4)]);
