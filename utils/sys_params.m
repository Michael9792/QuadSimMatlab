function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor
g = 9.81; % m/s/s

% old params
% m = 0.18; % kg
% I = [0.00025,   0,          2.55e-6;
%      0,         0.000232,   0;
%      2.55e-6,   0,          0.0003738];
% params.arm_length = 0.086; % m

% new params([51])
m = 4.34;
I = diag([0.0820 0.0845 0.1377]);
params.arm_length = 0.315;

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.minF = 0.0;
params.maxF = 2.0*m*g;

end
