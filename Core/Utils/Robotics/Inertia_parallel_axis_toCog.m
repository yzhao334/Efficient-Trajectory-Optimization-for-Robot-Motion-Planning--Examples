function [ IC ] = Inertia_parallel_axis_toCog( IR,M,d )
% parallel axis theorem of inertia of moment
% move inertia matrix from reference point to cog
% calculate inertia matrix relative to center of mass instead of a
% reference point R.
% input:
%    IR:   inertia matrix relative to a reference point R
%    M:    mass
%    d:    vector from center of mass to the reference point R
% output:
%    IC:   inertia matrix relative to center of mass
% author: Yu Zhao, yzhao334@berkeley.edu
%% useful function
hat=@(w)[0      -w(3)   w(2);...
         w(3)   0       -w(1);...
         -w(2)  w(1)    0];
IC = IR + M * (hat(d))^2;



end

