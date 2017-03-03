close all;
% clear all;
clearvars -global VarPhiGlobal eR MomentSave ForceSave Sum dot2;
% global VarPhiGlobal eR;
global MomentSave ForceSave Sum dot2;
addpath('utils');

%% pre-calculated trajectories
% trajhandle = @traj_line;
% trajhandle = @traj_helix;
% trajhandle = @traj_fixedpoint;
trajhandle = @traj_exciting;

%% Trajectory generation with waypoints
%% You need to implement this
% trajhandle = @traj_generator;
% waypoints = [0   0   0;
%              5    1.5   0;
%              10   2   0;
%              5   3   0;
%              0   4   0]';
% trajhandle([],[],waypoints);


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
plot(1:size(MomentSave, 1), MomentSave./ForceSave, 'b', 'linewidth', 1.5);
ylabel('$\frac{\bar{u}_2}{\bar{u}_1}$', 'interpreter','latex', 'FontSize', 20);

gravity = 9.81;
output1 = dot2(:, 3) + gravity;
output2 = dot2(:, 2) - dot2(:, 3) - gravity;
output3 = dot2(:, 1) - dot2(:, 3) - gravity;
[start_index, inv_mass]= RecursiveLeastSquareAnalysis(output1, Sum);
% [start_index, inv_mass]= RecursiveLeastSquareAnalysis(9.81, dot2, output1);
figure();
plot(1:size(Sum, 1), Sum./(dot2(3)+9.81), 'r', (start_index -1):(start_index +size(inv_mass, 1)) - 2, 1./inv_mass,  'b', 'linewidth', 1.5);
title('Comparison between direct calculation and RLS');
ylabel('estimated mass');
legend('Directly from Newton Formula', 'Recursive Least Square under Disturbance');