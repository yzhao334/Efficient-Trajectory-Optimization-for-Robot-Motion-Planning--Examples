function [ cost ] = costFcn( Xc,Uc,tc,D,nS,nU,ww,scale,bnds )
% cost function of time optimal control
cost = scale*2;% only consider trajectory time
% path cost
Xc = reshape(Xc,nS,numel(Xc)/nS);
Uc = reshape(Uc,nU,numel(Uc)/nU);
um1=bnds(1);um2=bnds(2);vm1=bnds(3);vm2=bnds(4);
uscale=diag([1/um1,1/um2])*Uc;
vscale=diag([1/vm1,1/vm2])*Xc(3:4,:);

cost = 0.5*cost + 0.25*sum(ww(:).'.*(0.5*sum(uscale.^2,1)...
    +0.5*sum((vscale.*uscale).^2,1)))*scale;




end

