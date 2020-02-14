function [ cost ] = costFcn( Xc,Uc,tc,D,nS,nU,ww,scale,rob )
% cost function of time optimal control
cost = scale*2;% only consider trajectory time
Xc = reshape(Xc,nS,numel(Xc)/nS);
Acc = (D*(Xc(7:12,:).')/scale).';
Jerk = (D*(Acc.')/scale).';

%% close to time optimal, with regulation
cost = cost ...                
            + 0.3*sum(ww(:).'.*( sum(diag(1./rob.r.G.^2)*...
                     Jerk.^2,1) ))*scale;

end

