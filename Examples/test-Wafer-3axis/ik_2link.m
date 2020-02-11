function[q1,q2]=ik_2link(x,y,side)
%----given the position of two link, and a side choice from 1 and -1
%----calculate q1 and q2 of the two link-------%
if side~=1 &&side~=-1
    disp('error: side must be 1 or -1');
    return;
end
l1=360;l2=440;
B_2=x^2+y^2;r=sqrt(B_2);
q2=real(side*acos((l1^2+l2^2-B_2)/(2*l1*l2)));%acos is from 0 to pi
phi2=acos((l1^2+B_2-l2^2)/(2*l1*r));
% q1=atan2(y,x)+side*phi2;
%----should consider the case that x=0, side can only be 1----%
q1=atan2(y,x)+side*phi2-pi/2;
k1=sin(q1);
k2=cos(q1);
q1=atan2(k1,k2);

