function [desired_state] = readytotransfer2(traj_time, t_real, waypoints0, X1, X2, X3)
%     the next line, syms function, really takes a very long time, so DO NOT USE IT DURING LOOP!
%     syms t


desired_state = struct('pos', zeros(3,1), 'vel', zeros(3,1), 'acc', zeros(3,1), 'yaw', 0, 'yawdot', 0);

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
        des_pos_x = waypoints0(1,1);
        des_pos_y = waypoints0(2,1)+4.5;
        des_pos_z = waypoints0(3,1);
        des_vel_x = 0;
        des_vel_y = 0;
        des_vel_z = 0;
        des_acc_x = 0;
        des_acc_y = 0;
        des_acc_z = 0;
    else
        des_pos_x = waypoints0(1,1);
        des_pos_y = waypoints0(2,1);
        des_pos_z = waypoints0(3,1);
        des_vel_x = 0;
        des_vel_y = 0;
        des_vel_z = 0;
        des_acc_x = 0;
        des_acc_y = 0;
        des_acc_z = 0;
    end
elseif(t_real>traj_time(1) && t_real<traj_time(2))
    des_pos_x = X1(1) + X1(2)*(t_real-S0)/(S1-S0) + X1(3)*((t_real-S0)/(S1-S0))^2 + X1(4)*((t_real-S0)/(S1-S0))^3 + X1(5)*((t_real-S0)/(S1-S0))^4 + X1(6)*((t_real-S0)/(S1-S0))^5 + X1(7)*((t_real-S0)/(S1-S0))^6 + X1(8)*((t_real-S0)/(S1-S0))^7;
    des_pos_y = X2(1) + X2(2)*(t_real-S0)/(S1-S0) + X2(3)*((t_real-S0)/(S1-S0))^2 + X2(4)*((t_real-S0)/(S1-S0))^3 + X2(5)*((t_real-S0)/(S1-S0))^4 + X2(6)*((t_real-S0)/(S1-S0))^5 + X2(7)*((t_real-S0)/(S1-S0))^6 + X2(8)*((t_real-S0)/(S1-S0))^7;
    des_pos_z = X3(1) + X3(2)*(t_real-S0)/(S1-S0) + X3(3)*((t_real-S0)/(S1-S0))^2 + X3(4)*((t_real-S0)/(S1-S0))^3 + X3(5)*((t_real-S0)/(S1-S0))^4 + X3(6)*((t_real-S0)/(S1-S0))^5 + X3(7)*((t_real-S0)/(S1-S0))^6 + X3(8)*((t_real-S0)/(S1-S0))^7;
    des_vel_x = X1(2)/(S1-S0) + X1(3)*2*(t_real-S0)/((S1-S0)^2) + X1(4)*3*((t_real-S0)^2)/((S1-S0)^3) + X1(5)*4*((t_real-S0)^3)/((S1-S0)^4) + X1(6)*5*((t_real-S0)^4)/((S1-S0)^5) + X1(7)*6*((t_real-S0)^5)/((S1-S0)^6) + X1(8)*7*((t_real-S0)^6)/((S1-S0)^7);
    des_vel_y = X2(2)/(S1-S0) + X2(3)*2*(t_real-S0)/((S1-S0)^2) + X2(4)*3*((t_real-S0)^2)/((S1-S0)^3) + X2(5)*4*((t_real-S0)^3)/((S1-S0)^4) + X2(6)*5*((t_real-S0)^4)/((S1-S0)^5) + X2(7)*6*((t_real-S0)^5)/((S1-S0)^6) + X2(8)*7*((t_real-S0)^6)/((S1-S0)^7);
    des_vel_z = X3(2)/(S1-S0) + X3(3)*2*(t_real-S0)/((S1-S0)^2) + X3(4)*3*((t_real-S0)^2)/((S1-S0)^3) + X3(5)*4*((t_real-S0)^3)/((S1-S0)^4) + X3(6)*5*((t_real-S0)^4)/((S1-S0)^5) + X3(7)*6*((t_real-S0)^5)/((S1-S0)^6) + X3(8)*7*((t_real-S0)^6)/((S1-S0)^7);
    des_acc_x = X1(3)*2/((S1-S0)^2) + X1(4)*3*2*(t_real-S0)/((S1-S0)^3) + X1(5)*4*3*((t_real-S0)^2)/((S1-S0)^4) + X1(6)*5*4*((t_real-S0)^3)/((S1-S0)^5) + X1(7)*6*5*((t_real-S0)^4)/((S1-S0)^6) + X1(8)*7*6*((t_real-S0)^5)/((S1-S0)^7);
    des_acc_y = X2(3)*2/((S1-S0)^2) + X2(4)*3*2*(t_real-S0)/((S1-S0)^3) + X2(5)*4*3*((t_real-S0)^2)/((S1-S0)^4) + X2(6)*5*4*((t_real-S0)^3)/((S1-S0)^5) + X2(7)*6*5*((t_real-S0)^4)/((S1-S0)^6) + X2(8)*7*6*((t_real-S0)^5)/((S1-S0)^7);
    des_acc_z = X3(3)*2/((S1-S0)^2) + X3(4)*3*2*(t_real-S0)/((S1-S0)^3) + X3(5)*4*3*((t_real-S0)^2)/((S1-S0)^4) + X3(6)*5*4*((t_real-S0)^3)/((S1-S0)^5) + X3(7)*6*5*((t_real-S0)^4)/((S1-S0)^6) + X3(8)*7*6*((t_real-S0)^5)/((S1-S0)^7);
elseif(t_real>=traj_time(2) && t_real<traj_time(3))
    des_pos_x = X1(1+8) + X1(2+8)*(t_real-S1)/(S2-S1) + X1(3+8)*((t_real-S1)/(S2-S1))^2 + X1(4+8)*((t_real-S1)/(S2-S1))^3 + X1(5+8)*((t_real-S1)/(S2-S1))^4 + X1(6+8)*((t_real-S1)/(S2-S1))^5 + X1(7+8)*((t_real-S1)/(S2-S1))^6 + X1(8+8)*((t_real-S1)/(S2-S1))^7;
    des_pos_y = X2(1+8) + X2(2+8)*(t_real-S1)/(S2-S1) + X2(3+8)*((t_real-S1)/(S2-S1))^2 + X2(4+8)*((t_real-S1)/(S2-S1))^3 + X2(5+8)*((t_real-S1)/(S2-S1))^4 + X2(6+8)*((t_real-S1)/(S2-S1))^5 + X2(7+8)*((t_real-S1)/(S2-S1))^6 + X2(8+8)*((t_real-S1)/(S2-S1))^7;
    des_pos_z = X3(1+8) + X3(2+8)*(t_real-S1)/(S2-S1) + X3(3+8)*((t_real-S1)/(S2-S1))^2 + X3(4+8)*((t_real-S1)/(S2-S1))^3 + X3(5+8)*((t_real-S1)/(S2-S1))^4 + X3(6+8)*((t_real-S1)/(S2-S1))^5 + X3(7+8)*((t_real-S1)/(S2-S1))^6 + X3(8+8)*((t_real-S1)/(S2-S1))^7;
    des_vel_x = X1(2+8)/(S2-S1) + X1(3+8)*2*(t_real-S1)/((S2-S1)^2) + X1(4+8)*3*((t_real-S1)^2)/((S2-S1)^3) + X1(5+8)*4*((t_real-S1)^3)/((S2-S1)^4) + X1(6+8)*5*((t_real-S1)^4)/((S2-S1)^5) + X1(7+8)*6*((t_real-S1)^5)/((S2-S1)^6) + X1(8+8)*7*((t_real-S1)^6)/((S2-S1)^7);
    des_vel_y = X2(2+8)/(S2-S1) + X2(3+8)*2*(t_real-S1)/((S2-S1)^2) + X2(4+8)*3*((t_real-S1)^2)/((S2-S1)^3) + X2(5+8)*4*((t_real-S1)^3)/((S2-S1)^4) + X2(6+8)*5*((t_real-S1)^4)/((S2-S1)^5) + X2(7+8)*6*((t_real-S1)^5)/((S2-S1)^6) + X2(8+8)*7*((t_real-S1)^6)/((S2-S1)^7);
    des_vel_z = X3(2+8)/(S2-S1) + X3(3+8)*2*(t_real-S1)/((S2-S1)^2) + X3(4+8)*3*((t_real-S1)^2)/((S2-S1)^3) + X3(5+8)*4*((t_real-S1)^3)/((S2-S1)^4) + X3(6+8)*5*((t_real-S1)^4)/((S2-S1)^5) + X3(7+8)*6*((t_real-S1)^5)/((S2-S1)^6) + X3(8+8)*7*((t_real-S1)^6)/((S2-S1)^7);
    des_acc_x = X1(3+8)*2/((S2-S1)^2) + X1(4+8)*3*2*(t_real-S1)/((S2-S1)^3) + X1(5+8)*4*3*((t_real-S1)^2)/((S2-S1)^4) + X1(6+8)*5*4*((t_real-S1)^3)/((S2-S1)^5) + X1(7+8)*6*5*((t_real-S1)^4)/((S2-S1)^6) + X1(8+8)*7*6*((t_real-S1)^5)/((S2-S1)^7);
    des_acc_y = X2(3+8)*2/((S2-S1)^2) + X2(4+8)*3*2*(t_real-S1)/((S2-S1)^3) + X2(5+8)*4*3*((t_real-S1)^2)/((S2-S1)^4) + X2(6+8)*5*4*((t_real-S1)^3)/((S2-S1)^5) + X2(7+8)*6*5*((t_real-S1)^4)/((S2-S1)^6) + X2(8+8)*7*6*((t_real-S1)^5)/((S2-S1)^7);
    des_acc_z = X3(3+8)*2/((S2-S1)^2) + X3(4+8)*3*2*(t_real-S1)/((S2-S1)^3) + X3(5+8)*4*3*((t_real-S1)^2)/((S2-S1)^4) + X3(6+8)*5*4*((t_real-S1)^3)/((S2-S1)^5) + X3(7+8)*6*5*((t_real-S1)^4)/((S2-S1)^6) + X3(8+8)*7*6*((t_real-S1)^5)/((S2-S1)^7);
elseif(t_real>=traj_time(3) && t_real<traj_time(4))
    des_pos_x = X1(1+16) + X1(2+16)*(t_real-S2)/(S3-S2) + X1(3+16)*((t_real-S2)/(S3-S2))^2 + X1(4+16)*((t_real-S2)/(S3-S2))^3 + X1(5+16)*((t_real-S2)/(S3-S2))^4 + X1(6+16)*((t_real-S2)/(S3-S2))^5 + X1(7+16)*((t_real-S2)/(S3-S2))^6 + X1(8+16)*((t_real-S2)/(S3-S2))^7;
    des_pos_y = X2(1+16) + X2(2+16)*(t_real-S2)/(S3-S2) + X2(3+16)*((t_real-S2)/(S3-S2))^2 + X2(4+16)*((t_real-S2)/(S3-S2))^3 + X2(5+16)*((t_real-S2)/(S3-S2))^4 + X2(6+16)*((t_real-S2)/(S3-S2))^5 + X2(7+16)*((t_real-S2)/(S3-S2))^6 + X2(8+16)*((t_real-S2)/(S3-S2))^7;
    des_pos_z = X3(1+16) + X3(2+16)*(t_real-S2)/(S3-S2) + X3(3+16)*((t_real-S2)/(S3-S2))^2 + X3(4+16)*((t_real-S2)/(S3-S2))^3 + X3(5+16)*((t_real-S2)/(S3-S2))^4 + X3(6+16)*((t_real-S2)/(S3-S2))^5 + X3(7+16)*((t_real-S2)/(S3-S2))^6 + X3(8+16)*((t_real-S2)/(S3-S2))^7;
    des_vel_x = X1(2+16)/(S3-S2) + X1(3+16)*2*(t_real-S2)/((S3-S2)^2) + X1(4+16)*3*((t_real-S2)^2)/((S3-S2)^3) + X1(5+16)*4*((t_real-S2)^3)/((S3-S2)^4) + X1(6+16)*5*((t_real-S2)^4)/((S3-S2)^5) + X1(7+16)*6*((t_real-S2)^5)/((S3-S2)^6) + X1(8+16)*7*((t_real-S2)^6)/((S3-S2)^7);
    des_vel_y = X2(2+16)/(S3-S2) + X2(3+16)*2*(t_real-S2)/((S3-S2)^2) + X2(4+16)*3*((t_real-S2)^2)/((S3-S2)^3) + X2(5+16)*4*((t_real-S2)^3)/((S3-S2)^4) + X2(6+16)*5*((t_real-S2)^4)/((S3-S2)^5) + X2(7+16)*6*((t_real-S2)^5)/((S3-S2)^6) + X2(8+16)*7*((t_real-S2)^6)/((S3-S2)^7);
    des_vel_z = X3(2+16)/(S3-S2) + X3(3+16)*2*(t_real-S2)/((S3-S2)^2) + X3(4+16)*3*((t_real-S2)^2)/((S3-S2)^3) + X3(5+16)*4*((t_real-S2)^3)/((S3-S2)^4) + X3(6+16)*5*((t_real-S2)^4)/((S3-S2)^5) + X3(7+16)*6*((t_real-S2)^5)/((S3-S2)^6) + X3(8+16)*7*((t_real-S2)^6)/((S3-S2)^7);
    des_acc_x = X1(3+16)*2/((S3-S2)^2) + X1(4+16)*3*2*(t_real-S2)/((S3-S2)^3) + X1(5+16)*4*3*((t_real-S2)^2)/((S3-S2)^4) + X1(6+16)*5*4*((t_real-S2)^3)/((S3-S2)^5) + X1(7+16)*6*5*((t_real-S2)^4)/((S3-S2)^6) + X1(8+16)*7*6*((t_real-S2)^5)/((S3-S2)^7);
    des_acc_y = X2(3+16)*2/((S3-S2)^2) + X2(4+16)*3*2*(t_real-S2)/((S3-S2)^3) + X2(5+16)*4*3*((t_real-S2)^2)/((S3-S2)^4) + X2(6+16)*5*4*((t_real-S2)^3)/((S3-S2)^5) + X2(7+16)*6*5*((t_real-S2)^4)/((S3-S2)^6) + X2(8+16)*7*6*((t_real-S2)^5)/((S3-S2)^7);
    des_acc_z = X3(3+16)*2/((S3-S2)^2) + X3(4+16)*3*2*(t_real-S2)/((S3-S2)^3) + X3(5+16)*4*3*((t_real-S2)^2)/((S3-S2)^4) + X3(6+16)*5*4*((t_real-S2)^3)/((S3-S2)^5) + X3(7+16)*6*5*((t_real-S2)^4)/((S3-S2)^6) + X3(8+16)*7*6*((t_real-S2)^5)/((S3-S2)^7);
elseif(t_real>=traj_time(4) && t_real<=traj_time(5))
    des_pos_x = X1(1+24) + X1(2+24)*(t_real-S3)/(S4-S3) + X1(3+24)*((t_real-S3)/(S4-S3))^2 + X1(4+24)*((t_real-S3)/(S4-S3))^3 + X1(5+24)*((t_real-S3)/(S4-S3))^4 + X1(6+24)*((t_real-S3)/(S4-S3))^5 + X1(7+24)*((t_real-S3)/(S4-S3))^6 + X1(8+24)*((t_real-S3)/(S4-S3))^7;
    des_pos_y = X2(1+24) + X2(2+24)*(t_real-S3)/(S4-S3) + X2(3+24)*((t_real-S3)/(S4-S3))^2 + X2(4+24)*((t_real-S3)/(S4-S3))^3 + X2(5+24)*((t_real-S3)/(S4-S3))^4 + X2(6+24)*((t_real-S3)/(S4-S3))^5 + X2(7+24)*((t_real-S3)/(S4-S3))^6 + X2(8+24)*((t_real-S3)/(S4-S3))^7;
    des_pos_z = X3(1+24) + X3(2+24)*(t_real-S3)/(S4-S3) + X3(3+24)*((t_real-S3)/(S4-S3))^2 + X3(4+24)*((t_real-S3)/(S4-S3))^3 + X3(5+24)*((t_real-S3)/(S4-S3))^4 + X3(6+24)*((t_real-S3)/(S4-S3))^5 + X3(7+24)*((t_real-S3)/(S4-S3))^6 + X3(8+24)*((t_real-S3)/(S4-S3))^7;
    des_vel_x = X1(2+24)/(S4-S3) + X1(3+24)*2*(t_real-S3)/((S4-S3)^2) + X1(4+24)*3*((t_real-S3)^2)/((S4-S3)^3) + X1(5+24)*4*((t_real-S3)^3)/((S4-S3)^4) + X1(6+24)*5*((t_real-S3)^4)/((S4-S3)^5) + X1(7+24)*6*((t_real-S3)^5)/((S4-S3)^6) + X1(8+24)*7*((t_real-S3)^6)/((S4-S3)^7);
    des_vel_y = X2(2+24)/(S4-S3) + X2(3+24)*2*(t_real-S3)/((S4-S3)^2) + X2(4+24)*3*((t_real-S3)^2)/((S4-S3)^3) + X2(5+24)*4*((t_real-S3)^3)/((S4-S3)^4) + X2(6+24)*5*((t_real-S3)^4)/((S4-S3)^5) + X2(7+24)*6*((t_real-S3)^5)/((S4-S3)^6) + X2(8+24)*7*((t_real-S3)^6)/((S4-S3)^7);
    des_vel_z = X3(2+24)/(S4-S3) + X3(3+24)*2*(t_real-S3)/((S4-S3)^2) + X3(4+24)*3*((t_real-S3)^2)/((S4-S3)^3) + X3(5+24)*4*((t_real-S3)^3)/((S4-S3)^4) + X3(6+24)*5*((t_real-S3)^4)/((S4-S3)^5) + X3(7+24)*6*((t_real-S3)^5)/((S4-S3)^6) + X3(8+24)*7*((t_real-S3)^6)/((S4-S3)^7);
    des_acc_x = X1(3+24)*2/((S4-S3)^2) + X1(4+24)*3*2*(t_real-S3)/((S4-S3)^3) + X1(5+24)*4*3*((t_real-S3)^2)/((S4-S3)^4) + X1(6+24)*5*4*((t_real-S3)^3)/((S4-S3)^5) + X1(7+24)*6*5*((t_real-S3)^4)/((S4-S3)^6) + X1(8+24)*7*6*((t_real-S3)^5)/((S4-S3)^7);
    des_acc_y = X2(3+24)*2/((S4-S3)^2) + X2(4+24)*3*2*(t_real-S3)/((S4-S3)^3) + X2(5+24)*4*3*((t_real-S3)^2)/((S4-S3)^4) + X2(6+24)*5*4*((t_real-S3)^3)/((S4-S3)^5) + X2(7+24)*6*5*((t_real-S3)^4)/((S4-S3)^6) + X2(8+24)*7*6*((t_real-S3)^5)/((S4-S3)^7);
    des_acc_z = X3(3+24)*2/((S4-S3)^2) + X3(4+24)*3*2*(t_real-S3)/((S4-S3)^3) + X3(5+24)*4*3*((t_real-S3)^2)/((S4-S3)^4) + X3(6+24)*5*4*((t_real-S3)^3)/((S4-S3)^5) + X3(7+24)*6*5*((t_real-S3)^4)/((S4-S3)^6) + X3(8+24)*7*6*((t_real-S3)^5)/((S4-S3)^7);
else
    des_pos_x = waypoints0(1,end);
    des_pos_y = waypoints0(2,end);
    des_pos_z = waypoints0(3,end);
    des_vel_x = 0;
    des_vel_y = 0;
    des_vel_z = 0;
    des_acc_x = 0;
    des_acc_y = 0;
    des_acc_z = 0;
end

desired_state.pos(1,1) = des_pos_x;
desired_state.pos(2,1) = des_pos_y;
desired_state.pos(3,1) = des_pos_z;
desired_state.vel(1,1) = des_vel_x;
desired_state.vel(2,1) = des_vel_y;
desired_state.vel(3,1) = des_vel_z;
desired_state.acc(1,1) = des_acc_x;
desired_state.acc(2,1) = des_acc_y;
desired_state.acc(3,1) = des_acc_z;
desired_state.yaw = 0;%  sin (t_real);
desired_state.yawdot = 0;% cos(t_real);

%     result = desired_state;
end