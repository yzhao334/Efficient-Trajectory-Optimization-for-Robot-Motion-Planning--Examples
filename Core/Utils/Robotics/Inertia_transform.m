function [ IC ] = Inertia_transform( ICB, M, T )
% Calculate inertia matrix relative to the origin of a given frame G.
% input:
%    ICB:  inertia matrix relative to center of mass in body frame B, B is
%    located in origin.
%    M:    mass
%    T:    transformation matrix between G and B, T = T_G^B.
%          If point p has coordinate y in frame B, and coordinate x in
%          frame G, then [x;1]=T[y;1]
% output:
%    IC:   inertia matrix relative to origin of G in frame G.
% author: Yu Zhao, yzhao334@berkeley.edu
A=T(1:3,1:3);
d=T(1:3,4);
IC1 = Inertia_body_frame(ICB,A);
IC = Inertia_parallel_axis(IC1,M,-d);


end

