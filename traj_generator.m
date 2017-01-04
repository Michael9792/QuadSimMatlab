function [ desired_state ] = traj_generator(t_real, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 traj_time d0
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     traj_time = [0, cumsum(d0)];
%     waypoints0 = waypoints;
% else
%     if(t > traj_time(end))
%         t = traj_time(end);
%     end
%     t_index = find(traj_time >= t,1);
%
%     if(t_index > 1)
%         t = t - traj_time(t_index-1);
%     end
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/d0(t_index-1);
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
%


%% Fill in your code here

persistent X1 X2 X3
persistent waypoints0 d0
global traj_time
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)/5];
    waypoints0 = waypoints;
    %    we should solve for x, y, z independently, we really need coefficients alpha_ij^x, alpha_ij^y, alpha_ij^z and they will be different.
    %    https://www.coursera.org/learn/robotics-flight/discussions/weeks/4/threads/7Ug8_GpJEeasOQpiYXGJHw
    
    syms t positive
    syms c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7 real
    syms c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7 real
    syms c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7 real
    syms c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7 real
    syms p1 p2 p3 p4
    S0 = traj_time(1);
    S1 = traj_time(2);
    S2 = traj_time(3);
    S3 = traj_time(4);
    S4 = traj_time(5);
    p1 = c_1_0 + c_1_1*(t-S0)/(S1-S0) + c_1_2*((t-S0)/(S1-S0))^2 + c_1_3*((t-S0)/(S1-S0))^3 + c_1_4*((t-S0)/(S1-S0))^4 + c_1_5*((t-S0)/(S1-S0))^5 + c_1_6*((t-S0)/(S1-S0))^6 + c_1_7*((t-S0)/(S1-S0))^7;
    p2 = c_2_0 + c_2_1*(t-S1)/(S2-S1) + c_2_2*((t-S1)/(S2-S1))^2 + c_2_3*((t-S1)/(S2-S1))^3 + c_2_4*((t-S1)/(S2-S1))^4 + c_2_5*((t-S1)/(S2-S1))^5 + c_2_6*((t-S1)/(S2-S1))^6 + c_2_7*((t-S1)/(S2-S1))^7;
    p3 = c_3_0 + c_3_1*(t-S2)/(S3-S2) + c_3_2*((t-S2)/(S3-S2))^2 + c_3_3*((t-S2)/(S3-S2))^3 + c_3_4*((t-S2)/(S3-S2))^4 + c_3_5*((t-S2)/(S3-S2))^5 + c_3_6*((t-S2)/(S3-S2))^6 + c_3_7*((t-S2)/(S3-S2))^7;
    p4 = c_4_0 + c_4_1*(t-S3)/(S4-S3) + c_4_2*((t-S3)/(S4-S3))^2 + c_4_3*((t-S3)/(S4-S3))^3 + c_4_4*((t-S3)/(S4-S3))^4 + c_4_5*((t-S3)/(S4-S3))^5 + c_4_6*((t-S3)/(S4-S3))^6 + c_4_7*((t-S3)/(S4-S3))^7;
    
    p1dot = diff(p1,t);
    p2dot = diff(p2,t);
    p3dot = diff(p3,t);
    p4dot = diff(p4,t);
    p1dot2 = diff(p1dot, t);
    p2dot2 = diff(p2dot, t);
    p3dot2 = diff(p3dot, t);
    p4dot2 = diff(p4dot, t);
    p1dot3 = diff(p1dot2, t);
    p2dot3 = diff(p2dot2, t);
    p3dot3 = diff(p3dot2, t);
    p4dot3 = diff(p4dot2, t);
    p1dot4 = diff(p1dot3, t);
    p2dot4 = diff(p2dot3, t);
    p3dot4 = diff(p3dot3, t);
    p4dot4 = diff(p4dot3, t);
    p1dot5 = diff(p1dot4, t);
    p2dot5 = diff(p2dot4, t);
    p3dot5 = diff(p3dot4, t);
    p4dot5 = diff(p4dot4, t);
    p1dot6 = diff(p1dot5, t);
    p2dot6 = diff(p2dot5, t);
    p3dot6 = diff(p3dot5, t);
    p4dot6 = diff(p4dot5, t);
    
    A = zeros(32, 32);
    b = zeros(32, 1);
    
    
    A(1, 1:8) = subs(jacobian(p1, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S0);
    A(2, 1:8) = subs(jacobian(p1, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S1);
    A(3, 9:16) = subs(jacobian(p2, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S1);
    A(4, 9:16) = subs(jacobian(p2, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S2);
    A(5, 17:24) = subs(jacobian(p3, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S2);
    A(6, 17:24) = subs(jacobian(p3, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S3);
    A(7, 25:32) = subs(jacobian(p4, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S3);
    A(8, 25:32) = subs(jacobian(p4, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S4);
    A(9, 1:8) = subs(jacobian(p1dot, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S0);
    A(10, 25:32) = subs(jacobian(p4dot, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S4);
    A(11, 1:8) = subs(jacobian(p1dot2, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S0);
    A(12, 25:32) = subs(jacobian(p4dot2, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S4);
    A(13, 1:8) = subs(jacobian(p1dot3, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S0);
    A(14, 25:32) = subs(jacobian(p4dot3, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S4);
    
    A(15, 1:8) = subs(jacobian(p1dot, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S1);
    A(15, 9:16) = -subs(jacobian(p2dot, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S1);
    A(16, 1:8) = subs(jacobian(p1dot2, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S1);
    A(16, 9:16) = -subs(jacobian(p2dot2, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S1);
    A(17, 1:8) = subs(jacobian(p1dot3, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S1);
    A(17, 9:16) = -subs(jacobian(p2dot3, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S1);
    A(18, 1:8) = subs(jacobian(p1dot4, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S1);
    A(18, 9:16) = -subs(jacobian(p2dot4, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S1);
    A(19, 1:8) = subs(jacobian(p1dot5, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S1);
    A(19, 9:16) = -subs(jacobian(p2dot5, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S1);
    A(20, 1:8) = subs(jacobian(p1dot6, [c_1_0 c_1_1 c_1_2 c_1_3 c_1_4 c_1_5 c_1_6 c_1_7]), t, S1);
    A(20, 9:16) = -subs(jacobian(p2dot6, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S1);
    
    A(21, 9:16) = subs(jacobian(p2dot, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S2);
    A(21, 17:24) = -subs(jacobian(p3dot, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S2);
    A(22, 9:16) = subs(jacobian(p2dot2, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S2);
    A(22, 17:24) = -subs(jacobian(p3dot2, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S2);
    A(23, 9:16) = subs(jacobian(p2dot3, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S2);
    A(23, 17:24) = -subs(jacobian(p3dot3, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S2);
    A(24, 9:16) = subs(jacobian(p2dot4, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S2);
    A(24, 17:24) = -subs(jacobian(p3dot4, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S2);
    A(25, 9:16) = subs(jacobian(p2dot5, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S2);
    A(25, 17:24) = -subs(jacobian(p3dot5, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S2);
    A(26, 9:16) = subs(jacobian(p2dot6, [c_2_0 c_2_1 c_2_2 c_2_3 c_2_4 c_2_5 c_2_6 c_2_7]), t, S2);
    A(26, 17:24) = -subs(jacobian(p3dot6, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S2);
    
    A(27, 17:24) = subs(jacobian(p3dot, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S3);
    A(27, 25:32) = -subs(jacobian(p4dot, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S3);
    A(28, 17:24) = subs(jacobian(p3dot2, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S3);
    A(28, 25:32) = -subs(jacobian(p4dot2, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S3);
    A(29, 17:24) = subs(jacobian(p3dot3, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S3);
    A(29, 25:32) = -subs(jacobian(p4dot3, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S3);
    A(30, 17:24) = subs(jacobian(p3dot4, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S3);
    A(30, 25:32) = -subs(jacobian(p4dot4, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S3);
    A(31, 17:24) = subs(jacobian(p3dot5, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S3);
    A(31, 25:32) = -subs(jacobian(p4dot5, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S3);
    A(32, 17:24) = subs(jacobian(p3dot6, [c_3_0 c_3_1 c_3_2 c_3_3 c_3_4 c_3_5 c_3_6 c_3_7]), t, S3);
    A(32, 25:32) = -subs(jacobian(p4dot6, [c_4_0 c_4_1 c_4_2 c_4_3 c_4_4 c_4_5 c_4_6 c_4_7]), t, S3);
    for k = 1:3
        for i = 1:(size(waypoints, 2)-1)
            b(2*i-1) = waypoints(k, i);
            b(2*i) = waypoints(k, i+1);
        end
        if(k == 1)
            X1 = A \ b;
        elseif(k==2)
            X2 = A \ b;
        elseif(k==3)
            X3 = A \ b;
        end
    end
else
    %     the next line, syms function, really takes a very long time, so DO NOT USE IT DURING LOOP!
    %     syms t
    S0 = traj_time(1);
    S1 = traj_time(2);
    S2 = traj_time(3);
    S3 = traj_time(4);
    S4 = traj_time(5);
    
    if(t_real >= traj_time(end))
        t_real = traj_time(end);
    end
    
    %   If I wanna change real start position, I can change the following
    %   error_attempt code to 1 or 0. and change waypoints() to plus
    %   whatever number means bias to real position.
    if t_real==traj_time(1)
        error_attempt = 0;
        if error_attempt == 1
            desired_state.pos(1,1) = waypoints0(1,1);
            desired_state.pos(2,1) = waypoints0(2,1)+4.5;
            desired_state.pos(3,1) = waypoints0(3,1);
            desired_state.vel(1,1) = 0;
            desired_state.vel(2,1) = 0;
            desired_state.vel(3,1) = 0;
            desired_state.acc(1,1) = 0;
            desired_state.acc(2,1) = 0;
            desired_state.acc(3,1) = 0;
        elseif error_attempt == 0
            desired_state.pos(1,1) = waypoints0(1,1);
            desired_state.pos(2,1) = waypoints0(2,1);
            desired_state.pos(3,1) = waypoints0(3,1);
            desired_state.vel(1,1) = 0;
            desired_state.vel(2,1) = 0;
            desired_state.vel(3,1) = 0;
            desired_state.acc(1,1) = 0;
            desired_state.acc(2,1) = 0;
            desired_state.acc(3,1) = 0;
        end
    elseif(t_real>traj_time(1) && t_real<traj_time(2))
        desired_state.pos(1,1) = X1(1) + X1(2)*(t_real-S0)/(S1-S0) + X1(3)*((t_real-S0)/(S1-S0))^2 + X1(4)*((t_real-S0)/(S1-S0))^3 + X1(5)*((t_real-S0)/(S1-S0))^4 + X1(6)*((t_real-S0)/(S1-S0))^5 + X1(7)*((t_real-S0)/(S1-S0))^6 + X1(8)*((t_real-S0)/(S1-S0))^7;
        desired_state.pos(2,1) = X2(1) + X2(2)*(t_real-S0)/(S1-S0) + X2(3)*((t_real-S0)/(S1-S0))^2 + X2(4)*((t_real-S0)/(S1-S0))^3 + X2(5)*((t_real-S0)/(S1-S0))^4 + X2(6)*((t_real-S0)/(S1-S0))^5 + X2(7)*((t_real-S0)/(S1-S0))^6 + X2(8)*((t_real-S0)/(S1-S0))^7;
        desired_state.pos(3,1) = X3(1) + X3(2)*(t_real-S0)/(S1-S0) + X3(3)*((t_real-S0)/(S1-S0))^2 + X3(4)*((t_real-S0)/(S1-S0))^3 + X3(5)*((t_real-S0)/(S1-S0))^4 + X3(6)*((t_real-S0)/(S1-S0))^5 + X3(7)*((t_real-S0)/(S1-S0))^6 + X3(8)*((t_real-S0)/(S1-S0))^7;
        desired_state.vel(1,1) = X1(2)/(S1-S0) + X1(3)*2*(t_real-S0)/((S1-S0)^2) + X1(4)*3*((t_real-S0)^2)/((S1-S0)^3) + X1(5)*4*((t_real-S0)^3)/((S1-S0)^4) + X1(6)*5*((t_real-S0)^4)/((S1-S0)^5) + X1(7)*6*((t_real-S0)^5)/((S1-S0)^6) + X1(8)*7*((t_real-S0)^6)/((S1-S0)^7);
        desired_state.vel(2,1) = X2(2)/(S1-S0) + X2(3)*2*(t_real-S0)/((S1-S0)^2) + X2(4)*3*((t_real-S0)^2)/((S1-S0)^3) + X2(5)*4*((t_real-S0)^3)/((S1-S0)^4) + X2(6)*5*((t_real-S0)^4)/((S1-S0)^5) + X2(7)*6*((t_real-S0)^5)/((S1-S0)^6) + X2(8)*7*((t_real-S0)^6)/((S1-S0)^7);
        desired_state.vel(3,1) = X3(2)/(S1-S0) + X3(3)*2*(t_real-S0)/((S1-S0)^2) + X3(4)*3*((t_real-S0)^2)/((S1-S0)^3) + X3(5)*4*((t_real-S0)^3)/((S1-S0)^4) + X3(6)*5*((t_real-S0)^4)/((S1-S0)^5) + X3(7)*6*((t_real-S0)^5)/((S1-S0)^6) + X3(8)*7*((t_real-S0)^6)/((S1-S0)^7);
        desired_state.acc(1,1) = X1(3)*2/((S1-S0)^2) + X1(4)*3*2*(t_real-S0)/((S1-S0)^3) + X1(5)*4*3*((t_real-S0)^2)/((S1-S0)^4) + X1(6)*5*4*((t_real-S0)^3)/((S1-S0)^5) + X1(7)*6*5*((t_real-S0)^4)/((S1-S0)^6) + X1(8)*7*6*((t_real-S0)^5)/((S1-S0)^7);
        desired_state.acc(2,1) = X2(3)*2/((S1-S0)^2) + X2(4)*3*2*(t_real-S0)/((S1-S0)^3) + X2(5)*4*3*((t_real-S0)^2)/((S1-S0)^4) + X2(6)*5*4*((t_real-S0)^3)/((S1-S0)^5) + X2(7)*6*5*((t_real-S0)^4)/((S1-S0)^6) + X2(8)*7*6*((t_real-S0)^5)/((S1-S0)^7);
        desired_state.acc(3,1) = X3(3)*2/((S1-S0)^2) + X3(4)*3*2*(t_real-S0)/((S1-S0)^3) + X3(5)*4*3*((t_real-S0)^2)/((S1-S0)^4) + X3(6)*5*4*((t_real-S0)^3)/((S1-S0)^5) + X3(7)*6*5*((t_real-S0)^4)/((S1-S0)^6) + X3(8)*7*6*((t_real-S0)^5)/((S1-S0)^7);
    elseif(t_real>=traj_time(2) && t_real<traj_time(3))
        desired_state.pos(1,1) = X1(1+8) + X1(2+8)*(t_real-S1)/(S2-S1) + X1(3+8)*((t_real-S1)/(S2-S1))^2 + X1(4+8)*((t_real-S1)/(S2-S1))^3 + X1(5+8)*((t_real-S1)/(S2-S1))^4 + X1(6+8)*((t_real-S1)/(S2-S1))^5 + X1(7+8)*((t_real-S1)/(S2-S1))^6 + X1(8+8)*((t_real-S1)/(S2-S1))^7;
        desired_state.pos(2,1) = X2(1+8) + X2(2+8)*(t_real-S1)/(S2-S1) + X2(3+8)*((t_real-S1)/(S2-S1))^2 + X2(4+8)*((t_real-S1)/(S2-S1))^3 + X2(5+8)*((t_real-S1)/(S2-S1))^4 + X2(6+8)*((t_real-S1)/(S2-S1))^5 + X2(7+8)*((t_real-S1)/(S2-S1))^6 + X2(8+8)*((t_real-S1)/(S2-S1))^7;
        desired_state.pos(3,1) = X3(1+8) + X3(2+8)*(t_real-S1)/(S2-S1) + X3(3+8)*((t_real-S1)/(S2-S1))^2 + X3(4+8)*((t_real-S1)/(S2-S1))^3 + X3(5+8)*((t_real-S1)/(S2-S1))^4 + X3(6+8)*((t_real-S1)/(S2-S1))^5 + X3(7+8)*((t_real-S1)/(S2-S1))^6 + X3(8+8)*((t_real-S1)/(S2-S1))^7;
        desired_state.vel(1,1) = X1(2+8)/(S2-S1) + X1(3+8)*2*(t_real-S1)/((S2-S1)^2) + X1(4+8)*3*((t_real-S1)^2)/((S2-S1)^3) + X1(5+8)*4*((t_real-S1)^3)/((S2-S1)^4) + X1(6+8)*5*((t_real-S1)^4)/((S2-S1)^5) + X1(7+8)*6*((t_real-S1)^5)/((S2-S1)^6) + X1(8+8)*7*((t_real-S1)^6)/((S2-S1)^7);
        desired_state.vel(2,1) = X2(2+8)/(S2-S1) + X2(3+8)*2*(t_real-S1)/((S2-S1)^2) + X2(4+8)*3*((t_real-S1)^2)/((S2-S1)^3) + X2(5+8)*4*((t_real-S1)^3)/((S2-S1)^4) + X2(6+8)*5*((t_real-S1)^4)/((S2-S1)^5) + X2(7+8)*6*((t_real-S1)^5)/((S2-S1)^6) + X2(8+8)*7*((t_real-S1)^6)/((S2-S1)^7);
        desired_state.vel(3,1) = X3(2+8)/(S2-S1) + X3(3+8)*2*(t_real-S1)/((S2-S1)^2) + X3(4+8)*3*((t_real-S1)^2)/((S2-S1)^3) + X3(5+8)*4*((t_real-S1)^3)/((S2-S1)^4) + X3(6+8)*5*((t_real-S1)^4)/((S2-S1)^5) + X3(7+8)*6*((t_real-S1)^5)/((S2-S1)^6) + X3(8+8)*7*((t_real-S1)^6)/((S2-S1)^7);
        desired_state.acc(1,1) = X1(3+8)*2/((S2-S1)^2) + X1(4+8)*3*2*(t_real-S1)/((S2-S1)^3) + X1(5+8)*4*3*((t_real-S1)^2)/((S2-S1)^4) + X1(6+8)*5*4*((t_real-S1)^3)/((S2-S1)^5) + X1(7+8)*6*5*((t_real-S1)^4)/((S2-S1)^6) + X1(8+8)*7*6*((t_real-S1)^5)/((S2-S1)^7);
        desired_state.acc(2,1) = X2(3+8)*2/((S2-S1)^2) + X2(4+8)*3*2*(t_real-S1)/((S2-S1)^3) + X2(5+8)*4*3*((t_real-S1)^2)/((S2-S1)^4) + X2(6+8)*5*4*((t_real-S1)^3)/((S2-S1)^5) + X2(7+8)*6*5*((t_real-S1)^4)/((S2-S1)^6) + X2(8+8)*7*6*((t_real-S1)^5)/((S2-S1)^7);
        desired_state.acc(3,1) = X3(3+8)*2/((S2-S1)^2) + X3(4+8)*3*2*(t_real-S1)/((S2-S1)^3) + X3(5+8)*4*3*((t_real-S1)^2)/((S2-S1)^4) + X3(6+8)*5*4*((t_real-S1)^3)/((S2-S1)^5) + X3(7+8)*6*5*((t_real-S1)^4)/((S2-S1)^6) + X3(8+8)*7*6*((t_real-S1)^5)/((S2-S1)^7);
    elseif(t_real>=traj_time(3) && t_real<traj_time(4))
        desired_state.pos(1,1) = X1(1+16) + X1(2+16)*(t_real-S2)/(S3-S2) + X1(3+16)*((t_real-S2)/(S3-S2))^2 + X1(4+16)*((t_real-S2)/(S3-S2))^3 + X1(5+16)*((t_real-S2)/(S3-S2))^4 + X1(6+16)*((t_real-S2)/(S3-S2))^5 + X1(7+16)*((t_real-S2)/(S3-S2))^6 + X1(8+16)*((t_real-S2)/(S3-S2))^7;
        desired_state.pos(2,1) = X2(1+16) + X2(2+16)*(t_real-S2)/(S3-S2) + X2(3+16)*((t_real-S2)/(S3-S2))^2 + X2(4+16)*((t_real-S2)/(S3-S2))^3 + X2(5+16)*((t_real-S2)/(S3-S2))^4 + X2(6+16)*((t_real-S2)/(S3-S2))^5 + X2(7+16)*((t_real-S2)/(S3-S2))^6 + X2(8+16)*((t_real-S2)/(S3-S2))^7;
        desired_state.pos(3,1) = X3(1+16) + X3(2+16)*(t_real-S2)/(S3-S2) + X3(3+16)*((t_real-S2)/(S3-S2))^2 + X3(4+16)*((t_real-S2)/(S3-S2))^3 + X3(5+16)*((t_real-S2)/(S3-S2))^4 + X3(6+16)*((t_real-S2)/(S3-S2))^5 + X3(7+16)*((t_real-S2)/(S3-S2))^6 + X3(8+16)*((t_real-S2)/(S3-S2))^7;
        desired_state.vel(1,1) = X1(2+16)/(S3-S2) + X1(3+16)*2*(t_real-S2)/((S3-S2)^2) + X1(4+16)*3*((t_real-S2)^2)/((S3-S2)^3) + X1(5+16)*4*((t_real-S2)^3)/((S3-S2)^4) + X1(6+16)*5*((t_real-S2)^4)/((S3-S2)^5) + X1(7+16)*6*((t_real-S2)^5)/((S3-S2)^6) + X1(8+16)*7*((t_real-S2)^6)/((S3-S2)^7);
        desired_state.vel(2,1) = X2(2+16)/(S3-S2) + X2(3+16)*2*(t_real-S2)/((S3-S2)^2) + X2(4+16)*3*((t_real-S2)^2)/((S3-S2)^3) + X2(5+16)*4*((t_real-S2)^3)/((S3-S2)^4) + X2(6+16)*5*((t_real-S2)^4)/((S3-S2)^5) + X2(7+16)*6*((t_real-S2)^5)/((S3-S2)^6) + X2(8+16)*7*((t_real-S2)^6)/((S3-S2)^7);
        desired_state.vel(3,1) = X3(2+16)/(S3-S2) + X3(3+16)*2*(t_real-S2)/((S3-S2)^2) + X3(4+16)*3*((t_real-S2)^2)/((S3-S2)^3) + X3(5+16)*4*((t_real-S2)^3)/((S3-S2)^4) + X3(6+16)*5*((t_real-S2)^4)/((S3-S2)^5) + X3(7+16)*6*((t_real-S2)^5)/((S3-S2)^6) + X3(8+16)*7*((t_real-S2)^6)/((S3-S2)^7);
        desired_state.acc(1,1) = X1(3+16)*2/((S3-S2)^2) + X1(4+16)*3*2*(t_real-S2)/((S3-S2)^3) + X1(5+16)*4*3*((t_real-S2)^2)/((S3-S2)^4) + X1(6+16)*5*4*((t_real-S2)^3)/((S3-S2)^5) + X1(7+16)*6*5*((t_real-S2)^4)/((S3-S2)^6) + X1(8+16)*7*6*((t_real-S2)^5)/((S3-S2)^7);
        desired_state.acc(2,1) = X2(3+16)*2/((S3-S2)^2) + X2(4+16)*3*2*(t_real-S2)/((S3-S2)^3) + X2(5+16)*4*3*((t_real-S2)^2)/((S3-S2)^4) + X2(6+16)*5*4*((t_real-S2)^3)/((S3-S2)^5) + X2(7+16)*6*5*((t_real-S2)^4)/((S3-S2)^6) + X2(8+16)*7*6*((t_real-S2)^5)/((S3-S2)^7);
        desired_state.acc(3,1) = X3(3+16)*2/((S3-S2)^2) + X3(4+16)*3*2*(t_real-S2)/((S3-S2)^3) + X3(5+16)*4*3*((t_real-S2)^2)/((S3-S2)^4) + X3(6+16)*5*4*((t_real-S2)^3)/((S3-S2)^5) + X3(7+16)*6*5*((t_real-S2)^4)/((S3-S2)^6) + X3(8+16)*7*6*((t_real-S2)^5)/((S3-S2)^7);
    elseif(t_real>=traj_time(4) && t_real<=traj_time(5))
        desired_state.pos(1,1) = X1(1+24) + X1(2+24)*(t_real-S3)/(S4-S3) + X1(3+24)*((t_real-S3)/(S4-S3))^2 + X1(4+24)*((t_real-S3)/(S4-S3))^3 + X1(5+24)*((t_real-S3)/(S4-S3))^4 + X1(6+24)*((t_real-S3)/(S4-S3))^5 + X1(7+24)*((t_real-S3)/(S4-S3))^6 + X1(8+24)*((t_real-S3)/(S4-S3))^7;
        desired_state.pos(2,1) = X2(1+24) + X2(2+24)*(t_real-S3)/(S4-S3) + X2(3+24)*((t_real-S3)/(S4-S3))^2 + X2(4+24)*((t_real-S3)/(S4-S3))^3 + X2(5+24)*((t_real-S3)/(S4-S3))^4 + X2(6+24)*((t_real-S3)/(S4-S3))^5 + X2(7+24)*((t_real-S3)/(S4-S3))^6 + X2(8+24)*((t_real-S3)/(S4-S3))^7;
        desired_state.pos(3,1) = X3(1+24) + X3(2+24)*(t_real-S3)/(S4-S3) + X3(3+24)*((t_real-S3)/(S4-S3))^2 + X3(4+24)*((t_real-S3)/(S4-S3))^3 + X3(5+24)*((t_real-S3)/(S4-S3))^4 + X3(6+24)*((t_real-S3)/(S4-S3))^5 + X3(7+24)*((t_real-S3)/(S4-S3))^6 + X3(8+24)*((t_real-S3)/(S4-S3))^7;
        desired_state.vel(1,1) = X1(2+24)/(S4-S3) + X1(3+24)*2*(t_real-S3)/((S4-S3)^2) + X1(4+24)*3*((t_real-S3)^2)/((S4-S3)^3) + X1(5+24)*4*((t_real-S3)^3)/((S4-S3)^4) + X1(6+24)*5*((t_real-S3)^4)/((S4-S3)^5) + X1(7+24)*6*((t_real-S3)^5)/((S4-S3)^6) + X1(8+24)*7*((t_real-S3)^6)/((S4-S3)^7);
        desired_state.vel(2,1) = X2(2+24)/(S4-S3) + X2(3+24)*2*(t_real-S3)/((S4-S3)^2) + X2(4+24)*3*((t_real-S3)^2)/((S4-S3)^3) + X2(5+24)*4*((t_real-S3)^3)/((S4-S3)^4) + X2(6+24)*5*((t_real-S3)^4)/((S4-S3)^5) + X2(7+24)*6*((t_real-S3)^5)/((S4-S3)^6) + X2(8+24)*7*((t_real-S3)^6)/((S4-S3)^7);
        desired_state.vel(3,1) = X3(2+24)/(S4-S3) + X3(3+24)*2*(t_real-S3)/((S4-S3)^2) + X3(4+24)*3*((t_real-S3)^2)/((S4-S3)^3) + X3(5+24)*4*((t_real-S3)^3)/((S4-S3)^4) + X3(6+24)*5*((t_real-S3)^4)/((S4-S3)^5) + X3(7+24)*6*((t_real-S3)^5)/((S4-S3)^6) + X3(8+24)*7*((t_real-S3)^6)/((S4-S3)^7);
        desired_state.acc(1,1) = X1(3+24)*2/((S4-S3)^2) + X1(4+24)*3*2*(t_real-S3)/((S4-S3)^3) + X1(5+24)*4*3*((t_real-S3)^2)/((S4-S3)^4) + X1(6+24)*5*4*((t_real-S3)^3)/((S4-S3)^5) + X1(7+24)*6*5*((t_real-S3)^4)/((S4-S3)^6) + X1(8+24)*7*6*((t_real-S3)^5)/((S4-S3)^7);
        desired_state.acc(2,1) = X2(3+24)*2/((S4-S3)^2) + X2(4+24)*3*2*(t_real-S3)/((S4-S3)^3) + X2(5+24)*4*3*((t_real-S3)^2)/((S4-S3)^4) + X2(6+24)*5*4*((t_real-S3)^3)/((S4-S3)^5) + X2(7+24)*6*5*((t_real-S3)^4)/((S4-S3)^6) + X2(8+24)*7*6*((t_real-S3)^5)/((S4-S3)^7);
        desired_state.acc(3,1) = X3(3+24)*2/((S4-S3)^2) + X3(4+24)*3*2*(t_real-S3)/((S4-S3)^3) + X3(5+24)*4*3*((t_real-S3)^2)/((S4-S3)^4) + X3(6+24)*5*4*((t_real-S3)^3)/((S4-S3)^5) + X3(7+24)*6*5*((t_real-S3)^4)/((S4-S3)^6) + X3(8+24)*7*6*((t_real-S3)^5)/((S4-S3)^7);
    end
    %     t_index = find(traj_time >= t,1);
    %
    %     if(t_index > 1)
    %         t = t - traj_time(t_index-1);
    %     end
    %     if(t == 0)
    %         desired_state.pos = waypoints0(:,1);
    %     else
    %         scale = t/d0(t_index-1);
    %         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    %     end
    %     desired_state.pos = (desired_state.pos)';
    %     desired_state.vel = zeros(3,1);
    %     desired_state.acc = zeros(3,1);
   desired_state.yaw = 0; %t_real/20; % 0;1.57; sin(t_real)
    desired_state.yawdot = 0;% cos(t_real);
end
% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end