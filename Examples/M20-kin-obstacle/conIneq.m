function [ ret ] = conIneq( Xc,Uc,D,scale,prob )
% inequality constraint
nS = 18;
nU = 6;
npts = numel(Xc)/nS;
Xc = reshape(Xc,nS,npts);
Uc = reshape(Uc,nU,npts);
Acc = Xc(13:18,:);
% Jerk = (D*(Acc.')/scale).';
% pos, velocity, torque constraints, considering motor feedforward
temp = [Xc;...
    Uc];


ret = [temp(:);...
    Xc(1:6,1);...
    Xc(7:12,1);...
    Acc(:,1);...
    prob.rob.tcpPos(Xc(1:6,end));...
    Xc(7:12,end);...
    Acc(:,end);...
    scale*2];

selfcoli=[];
wallcoli=[];
obscoli=[];
selfmap=prob.selfmap;
wallmap=prob.wallmap;
obsmap=prob.obsmap;
for i=1:npts
    prob.rob.jnt_pos=Xc(1:6,i);
    [jntT]=prob.rob.kfwd_rob_full();
    for j=[1,2,3,4,5]
        if isa(prob.rob.jnt_pos,'casadi.SX')
            temp=jntT{j}*[prob.c{j}.';ones(1,length(prob.r{j}))];
        else
            temp=jntT(:,:,j)*[prob.c{j}.';ones(1,length(prob.r{j}))];
        end
        c{j}=temp(1:3,:).';
    end
    
    for k=1:size(selfmap,2)
        lLind=selfmap(k).l(1);
        lBind=selfmap(k).l(2);
        rLind=selfmap(k).r(1);
        rBind=selfmap(k).r(2);
        selfcoli=[selfcoli;norm(c{lLind}(lBind,:)-c{rLind}(rBind,:))-(prob.r{lLind}(lBind)+prob.r{rLind}(rBind))];
    end
    for k=1:size(wallmap,2)
        Lind=wallmap(k).ind(1);
        Bind=wallmap(k).ind(2);
        wallcoli=[wallcoli;c{Lind}(Bind,3).']; % only consider z cood for ground
    end
    for k=1:size(obsmap,2)
        lLind=obsmap(k).l(1);
        lBind=obsmap(k).l(2);
        rBind=obsmap(k).r;
        obscoli=[obscoli;norm(c{lLind}(lBind,:)-prob.obs.c(rBind,:))-(prob.r{lLind}(lBind)+prob.obs.r(rBind))];
    end
end

ret=[ret;selfcoli];
ret=[ret;wallcoli];
ret=[ret;obscoli];


end

