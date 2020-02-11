function [ xyAcc ] = xyAcc( pos,vel,acc,rob )
% suppose each col of pos is at one time point
t1=pos(1,:);
t2=pos(2,:);
t3=pos(3,:);
t1d=vel(1,:);
t2d=vel(2,:);
t3d=vel(3,:);
t1dd=acc(1,:);
t2dd=acc(2,:);
t3dd=acc(3,:);
l1=rob.l1;
l2=rob.l2;
l3=rob.l3;

accx=...
    l2*(t1dd+t2dd).*cos(t1+t2)-(t1d+t2d).^2.*l2.*sin(t1+t2) ...
    -l1*cos(t1).*t1dd+l1*sin(t1).*t1d.^2 ...
    -l3*cos(t1+t2+t3).*(t1dd+t2dd+t3dd)+l3*sin(t1+t2+t3).*(t1d+t2d+t3d).^2;

accy=...
    -l1*sin(t1).*t1dd-l1*cos(t1).*t1d.^2+l2*sin(t1+t2).*(t1dd+t2dd) ...
    +l2*cos(t1+t2).*(t1d+t2d).^2-l3*sin(t1+t2+t3).*(t1dd+t2dd+t3dd) ...
    -l3*cos(t1+t2+t3).*(t1d+t2d+t3d).^2;
% xyAcc=sqrt(accx.^2+accy.^2);
xyAcc=[accx;accy];

end

