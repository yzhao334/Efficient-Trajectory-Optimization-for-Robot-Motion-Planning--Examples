function dX = robDyn( X,u,rob )
% simple 2d 2 link robot kinematics
% dX = robKin(X,u,rob)
% 
% This function computes simple 2d 2-link robot dynamics (scara)
% 
% INPUTS:
%   X = [4, n] = [Q;dQ] = state of the system
%           Q = [q1;q2], position of two joints
%          dQ = [dq1;dq2], velocity of two joints
%   u = [2, n] = [u1;u2] = control to the robot
%           u1 = torque of joint 1
%           u2 = torque of joint 2
alpha = rob.dyn.alpha;
beta = rob.dyn.beta;
delta = rob.dyn.delta;
c2 = cos(X(2,:));
s2 = sin(X(2,:));
d1 = X(3,:);
d2 = X(4,:);
u1 = u(1,:);u2 = u(2,:);
dX = [X(3:4,:);...
((- beta*s2.*d1.^2 + u2).*(delta + beta*c2) - delta*(u1 + beta*d2.*s2.*(d1 + d2) + beta*d1.*d2.*s2))./(beta^2*c2.^2 + delta^2 - alpha*delta);...
((delta + beta*c2).*(beta*s2.*d2.^2 + 2*beta*d1.*s2.*d2 + u1) - (- beta*s2.*d1.^2 + u2).*(alpha + 2*beta*c2))./(beta^2*c2.^2 + delta^2 - alpha*delta)...
];


end

