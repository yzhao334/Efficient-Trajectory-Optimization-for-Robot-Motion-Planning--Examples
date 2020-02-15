function [ cost ] = costFcn_topt( Xc,Uc,tc,D,nS,nU,ww,scale,rob )
%% cost function of 'pure' time optimal control
cost = scale*2;% only consider trajectory time

end

