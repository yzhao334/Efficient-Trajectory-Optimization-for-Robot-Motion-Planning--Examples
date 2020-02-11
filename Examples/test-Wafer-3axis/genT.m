function [T]=genT(the,pos)
%-----given the end position and theta, T matrix can generated 
%---the in degree----%
T=zeros(4,4);
the=the*pi/180;
T(1:2,1:2)=[cos(the) -sin(the);sin(the) cos(the)];
T(1,4)=pos(1);
T(2,4)=pos(2);
T(3,4)=pos(3);
T(4,4)=1;
