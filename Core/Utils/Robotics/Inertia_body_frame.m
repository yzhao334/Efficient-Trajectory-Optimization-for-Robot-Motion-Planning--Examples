function [ IC ] = Inertia_body_frame( ICB, A )
% Inertia matrix in different reference frames
% Calculate IC, the inertia matrix relative to center of mass in reference frame G
% remark: frame G and B must be located at the center of mass, only
% orientation can change.
% input:
%    ICB:  inertia matrix relative to center of mass in body frame B
%    A:    rotation matrix from frame B to G, A=R_G^B.
%          If vector p has coordinate y in frame B, and coordinate x in
%          frame G, then x=Ay.
% output:
%    IC:   inertia matrix relative to center of mass in frame G.
% author: Yu Zhao, yzhao334@berkeley.edu
IC = A*ICB*A.';


end

