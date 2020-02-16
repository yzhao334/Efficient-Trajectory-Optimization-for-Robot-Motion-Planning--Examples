function [ defects ] = conEq( Xc,Uc,D,nS,nU,scale,P,rob )
% equality constraint
Xc = reshape(Xc,nS,numel(Xc)/nS);
Uc = reshape(Uc,nU,numel(Uc)/nU);

defects = (D*(Xc.')/scale).';
defects = defects(:);

defects = defects - robDyn(Xc,Uc,rob);

end

