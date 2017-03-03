function [theta] = RecursiveLeastSquareAnalysis(param_gravity, dot2, param_inertialLift)
%% this function estimate mass, Fx, Fy under disturbances by recursive least square method. (RLS)
% init state
param_xdot2 = dot2(:, 1);
param_ydot2 = dot2(:, 2);
param_zdot2 = dot2(:, 3);
totalSize = size(param_xdot2, 1);
step = 100;
P = 1000000 * eye(3);
for i = 1:step:(totalSize - 1002 - step)
    xdot2 = param_xdot2(i:step:2*step+i);
    ydot2 = param_ydot2(i:step:2*step+i);
    zdot2 = param_zdot2(i:step:2*step+i);
% xdot2 = param_xdot2(1000+i+step);
% ydot2 = param_ydot2(1000+i+step);
% zdot2 = param_zdot2(1000+i+step);
inertialLift = param_inertialLift(i:step:2*step+i);
theta0 = zeros(size(inertialLift, 1), 1);
Phi = zeros(size(inertialLift, 1), 3);
Phi(:, 1) = inertialLift; 
Phi(:, 2) = 1; 
Phi(:, 3) = 1;

gravity = param_gravity * ones(size(inertialLift, 1), 1);
y = (zdot2 + gravity) +(ydot2 - zdot2 - gravity) + (xdot2-zdot2- gravity);
lambda = 0.9995;
theta = theta0;
% run
K = (P * Phi) /(lambda+Phi' * P * Phi);
error = y - Phi' * theta;
theta = theta + K * error;
P = (eye(size(K, 1)) - K * Phi') * P ./ lambda;
P = 0.5 * (P + P');
end
end