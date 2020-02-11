function [ ret ] = trans( q,x,y )
% transform matrices function
% q: rotation angle
% x,y: origin offset before rotation
ret = [[cos(q) sin(q);-sin(q) cos(q)].',[0;0];0,0,1]*[1 0 x;0 1 y;0 0 1];


end

