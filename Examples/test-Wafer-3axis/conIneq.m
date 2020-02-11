function [ ret ] = conIneq( Xc,Uc,D,scale,endpt,rob )
% inequality constraint
nS = 9;
nU = 3;
Xc = reshape(Xc,nS,numel(Xc)/nS);
Uc = reshape(Uc,nU,numel(Uc)/nU);
% jerk, velocity,acceleration constraints
temp = [Uc;Xc(nS/3+1:end,:)];
ret = [temp(:);Xc(nS/3+1:end,end);Xc(1:3,end)-endpt(:);scale*2];
% obstacle constrains
npts = size(Xc,2);
lens=[length(rob.link(1).c),length(rob.link(2).c),length(rob.link(3).c)];
nrob = sum(lens);
nobs = length(rob.obs.r);
conbndsX = [];
conbndsY = [];
conobs = [];
radiusS=[rob.link(1).r,rob.link(2).r,rob.link(3).r];
for k=1:npts
    T1=trans(Xc(1,k)+rob.joffset(1),rob.l1,0);
    T2=trans(Xc(2,k)+rob.joffset(2),rob.l2,0);
    T3=trans(Xc(3,k)+rob.joffset(3),rob.l3,0);
    temp1=[T1*[rob.link(1).c-rob.l1;zeros(1,lens(1));ones(1,lens(1))],...
           T1*T2*[rob.link(2).c-rob.l2;zeros(1,lens(2));ones(1,lens(2))],...
           T1*T2*T3*[rob.link(3).c-rob.l3;zeros(1,lens(3));ones(1,lens(3))]];
    conbndsX = [conbndsX;temp1(1,:).'];
    conbndsY = [conbndsY;temp1(2,:).'];
    for i=1:nrob
        for j=1:nobs
            conobs = [conobs; norm(temp1(1:2,i)-[rob.obs.cx(j);rob.obs.cy(j)])...
                - radiusS(i)-rob.obs.r(j)];
        end
    end
end
ret=[ret;conbndsX;conbndsY;conobs(:)];
acc=xyAcc(Xc(1:3,:),Xc(4:6,:),Xc(7:9,:),rob);
ret=[ret;acc(:)];


end

