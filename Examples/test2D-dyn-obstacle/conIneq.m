function [ ret ] = conIneq( Xc,Uc,D,scale,endpt,rob )
% inequality constraint
nS = 4;
nU = 2;
Xc = reshape(Xc,nS,numel(Xc)/nS);
Uc = reshape(Uc,nU,numel(Uc)/nU);
% velocity, torque constraints
temp = [Uc;Xc(3:4,:);...
    (D*(Xc(3:4,:).')/scale).';...
    (D^2*(Xc(3:4,:).')/scale^2).';...
    (D*(Uc.')/scale).'];
ret = [temp(:);Xc(3:4,end);Xc(1:2,end)-endpt(:);Uc(:,end);scale*2];
% obstacle constrains
npts = size(Xc,2);
nrob = length(rob.kin.l1pts)+length(rob.kin.l2pts);
nobs = size(rob.obs,1);
conobs = [];
len1=length(rob.kin.l1pts);
len2=length(rob.kin.l2pts);
for k=1:npts
    T1=trans(Xc(1,k),0,0);
    T2=trans(Xc(2,k),rob.kin.l1,0);
    temp1=[T1*[rob.kin.l1pts;zeros(1,len1);ones(1,len1)],...
           T1*T2*[rob.kin.l2pts;zeros(1,len2);ones(1,len2)]];
    for i=1:nrob
        for j=1:nobs
            conobs = [conobs; norm(temp1(1:2,i)-rob.obs(j,1:2).') - rob.kin.br-rob.obs(j,3)];
        end
    end
end
ret=[ret;conobs(:)];


end

