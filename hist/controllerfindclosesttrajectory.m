function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

%	This code includes how to find the closest trajectory by several vector and geometry relations.
%	According to paper "Trajectory Generation for quadrotors", it should own a much better controller performance.
%	However, it runs pretty slow. Guess caused by lots of vector calculation.


% =================== Your code goes here ===================
persistent last_desired_pos;
persistent last2_desired_pos;
Kpx = 20;
Kdx = 8;
Kpy = 20;
Kdy = 8;
Kpz = 30;
Kdz = 15;
Kpphi = 300;
Kdphi = 20;
Kptheta = 300;
Kdtheta = 20;
Kppsi = 200;
Kdpsi = 30;
desired_pos = [des_state.pos(1); des_state.pos(2); des_state.pos(3)];
real_pos_error = [des_state.pos(1) - state.pos(1); des_state.pos(2) - state.pos(2); des_state.pos(3) - state.pos(3)];
if(isempty(last_desired_pos) || isempty(last2_desired_pos))
    pos_error = real_pos_error;
else
    vector_r1 = last_desired_pos - last2_desired_pos;
    vector_tangent = desired_pos - last_desired_pos;
    if(norm(vector_r1)<inf || norm(vector_tangent)<inf)
        pos_error = real_pos_error;
    else
        unit_vector_tangent = vector_tangent / norm(vector_tangent);
        vector_temp = cross(vector_r1, unit_vector_tangent);
        unit_vector_temp=  vector_temp/ norm(vector_temp);
        
        vector_normal = cross(unit_vector_temp, unit_vector_tangent);
        unit_vector_normal = vector_normal / norm(vector_normal);
        vector_binormal = cross(unit_vector_tangent, unit_vector_normal);
        unit_vector_binormal = vector_binormal / norm(vector_binormal);
        
        pos_error = dot(real_pos_error, unit_vector_normal)*unit_vector_normal + dot(real_pos_error, unit_vector_binormal)*unit_vector_binormal;
    end
end
% state.acc()都被设置成了0
% pos_error(1) = des_state.pos(1) - state.pos(1);
% pos_error(2) = des_state.pos(2) - state.pos(2);
% pos_error(3) = des_state.pos(3) - state.pos(3);
r1_c_ddot = des_state.acc(1) + Kdx * (des_state.vel(1) - state.vel(1)) + Kpx * (pos_error(1));
r2_c_ddot = des_state.acc(2) + Kdy * (des_state.vel(2) - state.vel(2)) + Kpy * (pos_error(2));
r3_c_ddot = des_state.acc(3) + Kdz * (des_state.vel(3) - state.vel(3)) + Kpz * (pos_error(3));
r1_c_d3dot = Kdx * (des_state.acc(1)) + Kpx * (des_state.vel(1) - state.vel(1));
r2_c_d3dot = Kdy * (des_state.acc(2)) + Kpy * (des_state.vel(2) - state.vel(2));
r3_c_d3dot = Kdz * (des_state.acc(3)) + Kpz * (des_state.vel(3) - state.vel(3));


phic = (r1_c_ddot * sin(des_state.yaw) - r2_c_ddot * cos(des_state.yaw))/ params.gravity;
thetac = (r1_c_ddot * cos(des_state.yaw) + r2_c_ddot * sin(des_state.yaw))/ params.gravity;
psic = des_state.yaw;

phicdot = (r1_c_ddot * cos(des_state.yaw) * des_state.yawdot + r2_c_ddot * sin(des_state.yaw) * des_state.yawdot + r1_c_d3dot * sin(des_state.yaw) - r2_c_d3dot * cos(des_state.yaw))/ params.gravity;
thetacdot = (-r1_c_ddot * sin(des_state.yaw) * des_state.yawdot + r2_c_ddot * cos(des_state.yaw) * des_state.yawdot + r1_c_d3dot * cos(des_state.yaw) + r2_c_d3dot * sin(des_state.yaw))/ params.gravity;
psicdot = des_state.yawdot;


% Thrust
F = params.mass * (params.gravity + r3_c_ddot);

% Moment
M = zeros(3,1);
M(1) = params.I(1,1) * (Kdphi * (phicdot - state.omega(1)) + Kpphi * (phic - state.rot(1)));
M(2) = params.I(2,2) * (Kdtheta * (thetacdot - state.omega(2)) + Kptheta * (thetac - state.rot(2)));
M(3) = params.I(3,3) * (Kdpsi * (psicdot - state.omega(3)) + Kppsi * (psic - state.rot(3)));

last2_desired_pos = last_desired_pos;

last_desired_pos = [des_state.pos(1); des_state.pos(2); des_state.pos(3)];

% =================== Your code ends here ===================

end
