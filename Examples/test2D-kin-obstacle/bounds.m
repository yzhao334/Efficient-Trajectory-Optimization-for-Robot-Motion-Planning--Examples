function [ lb,ub ] = bounds( npts,rob )
% generate control lower bounds and upper bounds
conlb = -[10;10;2;2;5;5];
conub = [10;10;2;2;5;5];

lb = [kron(ones(npts,1),conlb);zeros(6,1);0];
ub = [kron(ones(npts,1),conub);zeros(6,1);100];

% bound for obstacle
nrob = length(rob.kin.l1pts)+length(rob.kin.l2pts);
nobs = size(rob.obs,1);

lb = [lb;0.05*ones(nrob*nobs*npts,1)];
ub = [ub;1000*ones(nrob*nobs*npts,1)];


end

