close all;
% clear all;
clearvars -global VarPhiGlobal eR;
global VarPhiGlobal eR;
addpath('utils');

%% pre-calculated trajectories
% trajhandle = @traj_line;
% trajhandle = @traj_helix;
trajhandle = @traj_fixedpoint;

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
figure();
plot(1:size(VarPhiGlobal, 1), VarPhiGlobal, 'b', 'linewidth', 1.5);
ylabel('\Phi');

figure();
plot(1:size(eR, 1), eR, 'b', 'linewidth', 1.5);
ylabel('e_R');