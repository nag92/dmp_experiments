% generates y = t^3 training trajectory
% from 0..1, with input dt as a step size
function [ T ] = y_cubed( dt )
t = 0:dt:1;
y = t.^3;
T(:,1) = y;
T(2:end,2) = diff(T(:,1), 1)/(dt^1);
T(3:end,3) = diff(T(:,1), 2)/(dt^2);
end