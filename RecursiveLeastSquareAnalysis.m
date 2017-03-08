function [start_index, theta, time_stamp] = RecursiveLeastSquareAnalysis(time, output, param_inertialLift)
%% this function estimate mass, Fx, Fy under disturbances by recursive least square method. (RLS)
% init state
totalSize = size(output, 1);
initial_value_step = 2;
step = 2;
P = 1000000 * eye(6);
estimated_param_count = size(param_inertialLift, 2);
theta = zeros(floor((totalSize - 5* initial_value_step -1)/step+1), estimated_param_count);
time_stamp = zeros(size(theta, 1), 1);
start_index = 5*step+1;
j = 1;
for i = 1:step:(totalSize - 5*initial_value_step)
    sliding_output = output(i:initial_value_step:5*initial_value_step+i);
    % xdot2 = param_xdot2(1000+i+step);
    % ydot2 = param_ydot2(1000+i+step);
    % zdot2 = param_zdot2(1000+i+step);
    inertialLift = param_inertialLift(i:initial_value_step:5*initial_value_step+i, :);
    % % Phi = zeros(size(inertialLift, 1), 3);
    Phi = inertialLift;
    y = sliding_output;
    lambda = 0.99;
    % run
    K = (P * Phi) /(lambda+Phi' * P * Phi + 1e-6*eye(estimated_param_count));
    error = y - Phi * theta(j, :)';
%     disp(theta(i, :));
    theta(j+1, :) = (theta(j, :)' + K' * error)';
    if theta(j+1, 1) > 1e3
        disp('estimated number is too large!');
        return;
    end
    time_stamp(j+1) = time(5*initial_value_step+i);
    P = (eye(size(K, 1)) - K * Phi') * P ./ lambda;
    P = 0.5 * (P + P');
    j = j+1;
end
end