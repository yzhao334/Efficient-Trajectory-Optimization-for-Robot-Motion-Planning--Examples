function [ lb,ub,bnds ] = bounds( npts,rob )
% generate control lower bounds and upper bounds
% vel bound +-2, acc bnd +-5, trq bnd j1 +-5, j2 +-5
% jerk bound +- 10, trq change rate +-20
conlb = [-5;-5;-2;-2;-5;-5;-10;-10;-20;-20];
conub = [5;5;2;2;5;5;10;10;20;20];
bnds = [5,2,2,2];% maximum torque and velocity

lb = [kron(ones(npts,1),conlb);zeros(4,1);zeros(2,1);0];
ub = [kron(ones(npts,1),conub);zeros(4,1);zeros(2,1);100];

% bound for obstacle
nrob = length(rob.kin.l1pts)+length(rob.kin.l2pts);
nobs = size(rob.obs,1);

lb = [lb;0.1*ones(nrob*nobs*npts,1)];
ub = [ub;1000*ones(nrob*nobs*npts,1)];


end

