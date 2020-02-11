function dX = robKin( X,u )
% dX = robKin(X,u,rob)
% 
% INPUTS:
%   X = [6, n] = [Q;dQ;ddQ] = state of the system
%   u = [2, n] = [u1;u2;u3] = control to the robot
n=3;
A = [zeros(n),eye(n),zeros(n);
     zeros(n),zeros(n),eye(n);
     zeros(n),zeros(n),zeros(n)];
B = [zeros(n);zeros(n);eye(n)];
dX = A*X + B*u;


end

