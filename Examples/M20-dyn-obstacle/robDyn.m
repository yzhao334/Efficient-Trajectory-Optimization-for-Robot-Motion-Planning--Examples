function dX = robDyn( X,u,rob )
% simple 6D robot dynamics
% dX = robKin(X,u,rob)
% 
% This function computes simple 2d 2-link robot dynamics (scara)
% 
% INPUTS:
%   X = [12, n] = [Q;dQ] = state of the system
%           Q = [q1;q2], position of two joints
%          dQ = [dq1;dq2], velocity of two joints
%   u = [6, n] = [u] = control to the robot
%           u = torque of joints

dX = rob.FDynVec( X, u );


end

