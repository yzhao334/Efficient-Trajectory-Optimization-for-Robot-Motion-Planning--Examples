function [ IR ] = Inertia_parallel_axis( IC,M,d )
% parallel axis theorem of inertia of moment
% move inertia matrix from cog to reference point
% calculate inertia matrix relative to a reference point R instead of
% center of mass
% input:
%    IC:   inertia matrix relative to center of mass
%    M:    mass
%    d:    vector from center of mass to the reference point R
% output:
%    IR:   inertia matrix relative to a reference point R
% author: Yu Zhao, yzhao334@berkeley.edu
%% useful function
hat=@(w)[0      -w(3)   w(2);...
         w(3)   0       -w(1);...
         -w(2)  w(1)    0];
IR = IC - M * (hat(d))^2;



end

