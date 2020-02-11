function [z,q1,q2,q3]=inverseKine(T,side)
l1=360;l2=440;l3=350;
rmax=1145.9;
z=T(3,4);
r=sqrt(T(1,4)^2+T(2,4)^2);
if r>rmax
    disp('robot out of range');
    return;
end
x_=T(1,4)-350*T(1,1);
y_=T(2,4)+350*T(1,2);
% [q1,q2]=inverseKine2(x_,y_);
% side=1;
[q1,q2]=ik_2link(x_,y_,side);
% q3=asin(T(2,1))-q1-q2+2*pi;
% q3=asin(T(2,1))-q1-q2+3*pi/2;
% k1=sin(q3);
% k2=cos(q3);
% q3=atan2(k1,k2);

t123=atan2(-T(2,2),-T(1,2));
q3=t123-q1-q2;
% temp=q1+q2+pi/2;
% q3=atan2(T(1,1),T(1,2))-temp;
% % q3=asin(T(1,1))-temp;
% if q3>190*pi/180
%     q3=q3-2*pi;
% elseif q3<-190*pi/180
%     q3=q3+2*pi;
% end
% out=[z,q1,q2,q3];
end

