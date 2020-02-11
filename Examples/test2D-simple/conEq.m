function [ defects ] = conEq( Xc,Uc,D,nS,nU,scale,P )
% equality constraint
Xc = reshape(Xc,nS,numel(Xc)/nS);
Uc = reshape(Uc,nU,numel(Uc)/nU);

defects = (D*(Xc.')/scale).' - robKin(Xc,Uc);
defects = defects(:);

end

