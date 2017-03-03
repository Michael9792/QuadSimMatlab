function [start_index, theta] = RecursiveLeastSquareAnalysis(output, param_inertialLift)
%% this function estimate mass, Fx, Fy under disturbances by recursive least square method. (RLS)
% init state
totalSize = size(output, 1);
step = 100;
P = 1000000 * eye(6);
theta = zeros(totalSize - 1001 - step, 1);
start_index = 6*step+1;
for i = 1:(totalSize - 6*step)
    sliding_output = output(i:step:5*step+i);
    % xdot2 = param_xdot2(1000+i+step);
    % ydot2 = param_ydot2(1000+i+step);
    % zdot2 = param_zdot2(1000+i+step);
    inertialLift = param_inertialLift(i:step:5*step+i);
    % % Phi = zeros(size(inertialLift, 1), 3);
    Phi = inertialLift;
    y = sliding_output;
    lambda = 0.99;
    % run
    K = (P * Phi) /(lambda+Phi' * P * Phi);
    error = y - Phi * theta(i);
    theta(i+1) = theta(i) + K' * error;
    P = (eye(size(K, 1)) - K * Phi') * P ./ lambda;
    P = 0.5 * (P + P');
end
end