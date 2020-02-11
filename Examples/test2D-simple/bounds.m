function [ lb,ub ] = bounds( npts )
% generate control lower bounds and upper bounds
conlb = -[10;10;2;2;5;5];
conub = [10;10;2;2;5;5];

lb = [kron(ones(npts,1),conlb);zeros(6,1);0];
ub = [kron(ones(npts,1),conub);zeros(6,1);100];


end

