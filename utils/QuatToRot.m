function R = QuatToRot(q)
%QuatToRot Converts a Quaternion to Rotation matrix
%   written by Daniel Mellinger

% normalize q
q = q./sqrt(sum(q.^2));

qahat(1,2) = -q(4);
qahat(1,3) = q(3);
qahat(2,3) = -q(2);
qahat(2,1) = q(4);
qahat(3,1) = -q(3);
qahat(3,2) = q(2);

% this equation is not the same as books I learned
% not the same as equations listed in Attitude Control Books
% not the same as equations listed in Mr. Geng's class.
% two equations just mentioned are not the same either. hahah .
R = eye(3) + 2*qahat*qahat + 2*q(1)*qahat;

end