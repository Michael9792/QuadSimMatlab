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


% =================== Your code goes here ===================
persistent atthist;
global VarPhiGlobal;
global eR;
global Moment
real_pos_error = [des_state.pos(1) - state.pos(1); des_state.pos(2) - state.pos(2); des_state.pos(3) - state.pos(3)];
pos_error = real_pos_error;
% state.acc()都被设置成了0
% pos_error(1) = des_state.pos(1) - state.pos(1);
% pos_error(2) = des_state.pos(2) - state.pos(2);
% pos_error(3) = des_state.pos(3) - state.pos(3);
largeanglecontrol = 1;
%% For small angle control. traditional control law.
if largeanglecontrol == 0
    %the following K parameters is for original model
%     Kpx = 20;
% Kdx = 8;
% Kpy = 20;
% Kdy = 8;
% Kpz = 30;
% Kdz = 15;
% Kpphi = 600;
% Kdphi = 40;
% Kptheta = 600;
% Kdtheta = 40;
% Kppsi = 200;
% Kdpsi = 30;% 200 30
% the following K params is for model in [51] thesis, not work for large
% attitude control
Kpx = 16;
Kdx = 5.6;
Kpy = 16;
Kdy = 5.6;
Kpz = 16;
Kdz = 5.6;
Kpphi = 8.81 / params.I(1,1);
Kdphi = 2.54 /  params.I(1,1);
Kptheta = 8.81 /  params.I(2, 2);
Kdtheta = 2.54 /  params.I(2,2);
Kppsi = 8.81/ params.I(3,3);
Kdpsi = 2.54 /  params.I(3,3);
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

atthist = [atthist [t; phic; thetac; psic; state.rot(1);  state.rot(2); state.rot(3)]];
% Thrust
F = params.mass * (params.gravity + r3_c_ddot);

% Moment
M = zeros(3,1);
M(1) = params.I(1,1) * (Kdphi * (phicdot - state.omega(1)) + Kpphi * (phic - state.rot(1)));
M(2) = params.I(2,2) * (Kdtheta * (thetacdot - state.omega(2)) + Kptheta * (thetac - state.rot(2)));
M(3) = params.I(3,3) * (Kdpsi * (psicdot - state.omega(3)) + Kppsi * (psic - state.rot(3)));

elseif largeanglecontrol == 1
%% For large angle control

%% original quadrotor model parameters
% Kpx = 20;
% Kdx = 8;
% Kpy = 20;
% Kdy = 8;
% Kpz = 30;
% Kdz = 15;
% Kpphi = 100;
% Kdphi = 20;
% Kptheta = 100;
% Kdtheta = 20;
% Kppsi = 40;
% Kdpsi = 100;

%% params for model in [51] thesis
Kpx = 16;
Kdx = 5.6;
Kpy = 16;
Kdy = 5.6;
Kpz = 16;
Kdz = 5.6;

%% stiff attitude control parameter(almost flipped attitude(165 \degree), stable to fixed point)
% if oscilation is large, just decrease Kd. Strange.
Kpphi = 60 / params.I(1,1);
Kdphi = 6 /  params.I(1,1);
Kptheta = 60 /  params.I(2, 2);
Kdtheta = 6 /  params.I(2,2);
Kppsi = 8.81/ params.I(3,3);
Kdpsi = 2.54 /  params.I(3,3);

%% soft attitude control parameter(3D trajectory tracking)
% Kpphi = 30 / params.I(1,1);
% Kdphi = 2.54 /  params.I(1,1);
% Kptheta = 30 /  params.I(2, 2);
% Kdtheta = 2.54 /  params.I(2,2);
% Kppsi = 8.81/ params.I(3,3);
% Kdpsi = 2.54 /  params.I(3,3);

%% bad attitude control parameter(stated in [51])
% Kpphi = 8.81 / params.I(1,1);
% Kdphi = 2.54 /  params.I(1,1);
% Kptheta = 8.81 /  params.I(2, 2);
% Kdtheta = 2.54 /  params.I(2,2);
% Kppsi = 8.81/ params.I(3,3);
% Kdpsi = 2.54 /  params.I(3,3);

r1_c_ddot = des_state.acc(1) + Kdx * (des_state.vel(1) - state.vel(1)) + Kpx * (pos_error(1));
r2_c_ddot = des_state.acc(2) + Kdy * (des_state.vel(2) - state.vel(2)) + Kpy * (pos_error(2));
r3_c_ddot = des_state.acc(3) + Kdz * (des_state.vel(3) - state.vel(3)) + Kpz * (pos_error(3));
r1_c_d3dot = Kdx * (des_state.acc(1)) + Kpx * (des_state.vel(1) - state.vel(1));
r2_c_d3dot = Kdy * (des_state.acc(2)) + Kpy * (des_state.vel(2) - state.vel(2));
r3_c_d3dot = Kdz * (des_state.acc(3)) + Kpz * (des_state.vel(3) - state.vel(3));

phicdot = (r1_c_ddot * cos(des_state.yaw) * des_state.yawdot + r2_c_ddot * sin(des_state.yaw) * des_state.yawdot + r1_c_d3dot * sin(des_state.yaw) - r2_c_d3dot * cos(des_state.yaw))/ params.gravity;
thetacdot = (-r1_c_ddot * sin(des_state.yaw) * des_state.yawdot + r2_c_ddot * cos(des_state.yaw) * des_state.yawdot + r1_c_d3dot * cos(des_state.yaw) + r2_c_d3dot * sin(des_state.yaw))/ params.gravity;
psicdot = des_state.yawdot;

F_des_x_W = params.mass * r1_c_ddot;
F_des_y_W= params.mass * r2_c_ddot;
F_des_z_W = params.mass * (r3_c_ddot + params.gravity);

R = [(cos(state.rot(2)) * cos(state.rot(3)) - sin(state.rot(1)) * sin(state.rot(3)) * sin(state.rot(2))) -(cos(state.rot(1)) * sin(state.rot(3))) (cos(state.rot(3)) * sin(state.rot(2)) + cos(state.rot(2)) * sin(state.rot(1)) * sin(state.rot(3)));...
(cos(state.rot(2)) * sin(state.rot(3)) + cos(state.rot(3)) * sin(state.rot(1)) * sin(state.rot(2))) cos(state.rot(1)) * cos(state.rot(3)) (sin(state.rot(3)) * sin(state.rot(2)) - cos(state.rot(3)) * cos(state.rot(2)) * sin(state.rot(1)));...
-cos(state.rot(1)) * sin(state.rot(2)) sin(state.rot(1)) (cos(state.rot(1)) * cos(state.rot(2)))];

F_des_x_B = F_des_x_W * (cos(state.rot(2)) * cos(state.rot(3)) - sin(state.rot(1)) * sin(state.rot(3)) * sin(state.rot(2))) + F_des_y_W * (cos(state.rot(2)) * sin(state.rot(3)) + cos(state.rot(3)) * sin(state.rot(1)) * sin(state.rot(2))) - F_des_z_W * cos(state.rot(1)) * sin(state.rot(2));
F_des_y_B = -F_des_x_W* (cos(state.rot(1)) * sin(state.rot(3))) + F_des_y_W * cos(state.rot(1)) * cos(state.rot(3)) + F_des_z_W * sin(state.rot(1));
F_des_z_B = F_des_x_W * (cos(state.rot(3)) * sin(state.rot(2)) + cos(state.rot(2)) * sin(state.rot(1)) * sin(state.rot(3))) + F_des_y_W * (sin(state.rot(3)) * sin(state.rot(2)) - cos(state.rot(3)) * cos(state.rot(2)) * sin(state.rot(1))) + F_des_z_W * (cos(state.rot(1)) * cos(state.rot(2)));

u1_des = F_des_z_B;

% z_B_des_W = [F_des_x_B; F_des_y_B; F_des_z_B] / norm([F_des_x_B; F_des_y_B; F_des_z_B]);
z_B_des_W = [F_des_x_W; F_des_y_W; F_des_z_W] / norm([F_des_x_W; F_des_y_W; F_des_z_W]);

X_C_des_W = [cos(des_state.yaw); sin(des_state.yaw); 0];
temp = cross(z_B_des_W, X_C_des_W);
y_B_des_W = temp / norm(temp);
x_B_des_W = cross(y_B_des_W, z_B_des_W);
R_des = [x_B_des_W y_B_des_W z_B_des_W];
% R_des = [cos(thetac) 0 sin(thetac); 0 1 0;-sin(thetac) 0 cos(thetac)];
% R_des = [cos(psic) -sin(psic) 0; sin(psic) cos(psic) 0; 0 0 1];

%%%%% some problem with this part %%%%%
% phic and thetac is not right, sometimes.
phic = asin(-R_des(3, 2));
thetac = atan2(-R_des(3,1) , R_des(3,3));
psic = atan2(-R_des(1,2), R_des(2,2));

%%%%% some problem with this part %%%%%
atthist = [atthist [t; phic; thetac; psic; state.rot(1);  state.rot(2); state.rot(3)]];

% e_R is vee map which takes elements of so(3) to R3. 
% it turns a skew-symmetric matrix to a vector
% 
%%
% <https://github.com/justinthomas/MATLAB-tools/blob/master/vee.m>
e_R = 0.5 * vee((R_des' * R - R' * R_des)); 
VarPhi = 0.5 * trace(diag([1 1 1]) - R_des' * R);
VarPhiGlobal = [VarPhiGlobal; VarPhi];
eR = [eR; e_R(1)];
e_w = [phicdot - state.omega(1); thetacdot - state.omega(2); (psicdot - state.omega(3))];

%%%%% Not completed Start%%%%%
% K_R, K_w might be redesigned.
real_kR = diag([Kpphi * params.I(1,1); Kptheta * params.I(2,2); Kppsi * params.I(3,3)]);
real_kW = diag([Kdphi * params.I(1,1); Kdtheta * params.I(2,2); Kdpsi * params.I(3,3)]);
%%%%% Not completed Finish%%%%%

%   error tolerance
if max(abs(e_R)) < 1e-3 && max(abs(e_w)) < 1e-3
    u2_des = [0;0;0];
else
    u2_des =  -real_kR * e_R + real_kW * e_w; 
end

% Thrust
F = u1_des;

% Moment
M = u2_des;

% After several seconds, the quad receives control signal and start to
% stablize. This is just for plotting, no actual improvements to control
% algorithms. 
if t<0.2
    F = 0;
    M = zeros(3,1);
    
end
Moment = [Moment [ t; F; M]];
% =================== Your code ends here ===================
end
end
