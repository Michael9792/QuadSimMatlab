close all;
% clear all;
clearvars -global VarPhiGlobal eR MomentSave ForceSave lift_force dot2 qdot_save;
% global VarPhiGlobal eR;
global MomentSave ForceSave lift_force dot2 qdot_save;
addpath('utils');

%% pre-calculated trajectories
% trajhandle = @traj_line;
% trajhandle = @traj_helix;
% trajhandle = @traj_fixedpoint;
% trajhandle = @traj_exciting;

%% Trajectory generation with waypoints
%% You need to implement this
trajhandle = @traj_generator;
waypoints = [0   0   0;
             5    0   0;
             0   0   0;
             5   2   0;
             -1   3   0]';
trajhandle([],[],waypoints);


%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);

%% Post Evaluation
% figure();
% plot(1:size(VarPhiGlobal, 1), VarPhiGlobal, 'b', 'linewidth', 1.5);
% ylabel('\Phi');
% 
% figure();
% plot(1:size(eR, 1), eR, 'b', 'linewidth', 1.5);
% ylabel('e_R');

figure();
plot(1:size(MomentSave, 1), MomentSave(:, 1)./ForceSave, 'b', 'linewidth', 1.5);
ylabel('$\frac{\bar{u}_2}{\bar{u}_1}$', 'interpreter','latex', 'FontSize', 20);

gravity = 9.81;
output1 = dot2(:, 4) + gravity;
Time = lift_force(:, 1);
Phi_mass = lift_force(:, 4);
[start_index, inv_mass, time_stamp]= RecursiveLeastSquareAnalysis(Time, output1, Phi_mass);
figure();
plot(lift_force(:, 1), lift_force(:, 4)./(dot2(:, 4)+9.81), 'r', time_stamp, 1./inv_mass,  'b', 'linewidth', 1.5);
title('Comparison between real mass and RLS');
ylabel('estimated mass');
legend('Real value', 'Recursive Least Square under Disturbance');

% output2 = 4.34 * dot2(:, 1) - lift_force(:, 1);
% Phi_Fx = ones(size(lift_force, 1), 1);
% [start_index, Fx]= RecursiveLeastSquareAnalysis(output2, Phi_Fx);
% figure();
% plot((start_index -1):(start_index +size(Fx, 1) - 2), Fx,  'b', 'linewidth', 1.5);
% title('Disturbance Force in X direction');
% ylabel('estimated Force');
% 
% output3 = 4.34 * dot2(:, 2) - lift_force(:, 2);
% Phi_Fy = ones(size(lift_force, 1), 1);
% [start_index, Fy]= RecursiveLeastSquareAnalysis(output3, Phi_Fy);
% figure();
% plot((start_index -1):(start_index +size(Fy, 1) - 2), Fy,  'b', 'linewidth', 1.5);
% title('Disturbance Force in Y direction');
% ylabel('estimated Force');

Phi_u3 = MomentSave(:, 2);
Phi_u1 = ForceSave;
Time = lift_force(:, 1);
[start_index, theta, time_stamp]= RecursiveLeastSquareAnalysis(Time, qdot_save, [Phi_u3 Phi_u1]);
Iyy = 1./theta(:, 1);
xoff = theta(:, 2) .* theta(:, 1);
inv_Iyy_multiply_xoff=theta(:, 2);
% fault detection
indice = abs(Iyy)>10;
Iyy(indice) = 0;
indice = abs(xoff)>10;
xoff(indice) = 0;

figure();
plot(time_stamp, Iyy, 'b', 'linewidth', 1.5);
title('I_{yy}');
ylabel('estimated inertia');

figure();
plot(time_stamp, xoff, 'b', 'linewidth', 1.5);
title('x_{off}');
ylabel('estimated x_{off}');