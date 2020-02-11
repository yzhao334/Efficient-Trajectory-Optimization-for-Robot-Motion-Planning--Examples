function [ xy ] = forwardKine( q,rob )
% forward kinematics
n=size(q,1);
xy=nan(n,2);
for k=1:n
    T1=trans(q(k,1)+rob.joffset(1),rob.l1,0);
    T2=trans(q(k,2)+rob.joffset(2),rob.l2,0);
    T3=trans(q(k,3)+rob.joffset(3),rob.l3,0);
    temp=T1*T2;
    T=temp*T3;
    xy(k,:,1)=[T1(1,3),T1(2,3)];
    xy(k,:,2)=[temp(1,3),temp(2,3)];
    xy(k,:,3)=[T(1,3),T(2,3)];
end


end

