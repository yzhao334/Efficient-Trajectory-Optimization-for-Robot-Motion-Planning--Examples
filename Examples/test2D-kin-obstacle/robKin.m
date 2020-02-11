function dX = robKin( X,u )
% simple 2d 2 link robot kinematics
% dX = robKin(X,u,rob)
% 
% This function computes simple 2d 2-link robot kinematics (scara)
% 
% INPUTS:
%   X = [6, n] = [Q;dQ;ddQ] = state of the system
%           Q = [q1;q2], position of two joints
%          dQ = [dq1;dq2], velocity of two joints
%         ddQ = [ddq1;ddq2], acceleration of two joints
%   u = [2, n] = [u1;u2] = control to the robot
%           u1 = jerk of joint 1
%           u2 = jerk of joint 2
A = [zeros(2),eye(2),zeros(2);
     zeros(2),zeros(2),eye(2);
     zeros(2),zeros(2),zeros(2)];
B = [zeros(2);zeros(2);eye(2)];
dX = A*X + B*u;


end

