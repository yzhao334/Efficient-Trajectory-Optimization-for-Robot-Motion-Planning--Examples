function dX = robDyn( X,u,rob )
% simple 6D robot kinematics
n=6;
A = [zeros(n),eye(n),zeros(n);
     zeros(n),zeros(n),eye(n);
     zeros(n),zeros(n),zeros(n)];
B = [zeros(n);zeros(n);eye(n)];
dX = A*X + B*u;
dX=dX(:);



end

