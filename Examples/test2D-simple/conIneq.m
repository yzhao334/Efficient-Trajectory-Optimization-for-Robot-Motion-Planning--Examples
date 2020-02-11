function [ ret ] = conIneq( Xc,Uc,D,scale,endpt )
% inequality constraint
nS = 6;
nU = 2;
Xc = reshape(Xc,nS,numel(Xc)/nS);
Uc = reshape(Uc,nU,numel(Uc)/nU);
temp = [Uc;Xc(3:end,:)];

ret = [temp(:);Xc(nS/3+1:end,end);Xc(1:2,end)-endpt(:);scale*2];


end

