function [ traj_time, X1, X2, X3 ] = traj_generator_convert(t_real, state, waypoints)
	d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)/3];
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
end