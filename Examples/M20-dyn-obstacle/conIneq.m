function [ ret ] = conIneq( Xc,Uc,D,scale,prob )
% inequality constraint
nS = 12;
nU = 6;
npts = numel(Xc)/nS;
Xc = reshape(Xc,nS,npts);
Uc = reshape(Uc,nU,npts);
Acc = (D*(Xc(7:12,:).')/scale).';
Jerk = (D*(Acc.')/scale).';
Ucd = (D*(Uc.')/scale).';
% pos, velocity, torque constraints, considering motor feedforward
temp = [Xc;...
    Uc+diag(prob.rob.r.Jm.*prob.rob.r.G.^2)*Acc;...
    Ucd+diag(prob.rob.r.Jm.*prob.rob.r.G.^2)*Jerk];

ret = [temp(:);...
    Xc(1:6,1);...
    Xc(7:12,1);...
    Acc(:,1);...
    Xc(1:6,end);...
    Xc(7:12,end);...
    Acc(:,end);...
    scale*2];

selfcoli=[];
wallcoli=[];
obscoli=[];
c=prob.bc;
r=prob.br;
selfmap=prob.selfmap;
wallmap=prob.wallmap;
obsmap=prob.obsmap;
for i=1:npts
    [jntT]=kfwd_rob_m16R(Xc(1:6,i),prob.si.DH,prob.rob.rtb.tool);
    for j=[1,2,3,4,5,8]
        temp=jntT{j}*[prob.bc{j}.';ones(1,length(r{j}))];
        c{j}=temp(1:3,:).';
    end
    for k=1:size(selfmap,2)
        lLind=selfmap(k).l(1);
        lBind=selfmap(k).l(2);
        rLind=selfmap(k).r(1);
        rBind=selfmap(k).r(2);
        selfcoli=[selfcoli;norm(c{lLind}(lBind,:)-c{rLind}(rBind,:))-(r{lLind}(lBind)+r{rLind}(rBind))];
    end
    for k=1:size(wallmap,2)
        Lind=wallmap(k).ind(1);
        Bind=wallmap(k).ind(2);
        wallcoli=[wallcoli;c{Lind}(Bind,:).'];
    end
    for k=1:size(obsmap,2)
        lLind=obsmap(k).l(1);
        lBind=obsmap(k).l(2);
        rBind=obsmap(k).r;
        obscoli=[obscoli;norm(c{lLind}(lBind,:)-prob.obs.c(rBind,:))-(r{lLind}(lBind)+prob.obs.r(rBind))];
    end
end

ret=[ret;selfcoli];
ret=[ret;wallcoli];
ret=[ret;obscoli];


end

