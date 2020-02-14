function [ cost ] = costFcn( Xc,Uc,tc,D,nS,nU,ww,scale )
% cost function of time optimal control
cost = scale*2;% only consider trajectory time
% W=175*diag(1./[174.5625
%   175.2857
%   191.3360
%    22.5844
%    68.6337
%    50.0000]);
Xc = reshape(Xc,nS,numel(Xc)/nS);
%xtune = Xc(1:nS/2,:) - kron(ones(1,numel(Xc)/nS),target(:));
xtune = Xc(1:nS/2,:) - kron(ones(1,numel(Xc)/nS),[pi/3,pi/3,-pi/4,0,-pi/3,0].');
Uc = reshape(Uc,nU,numel(Uc)/nU);
utune = Uc/100;
% utune = Uc;

% cost = 0.3*cost + 0.7*sum(tc(:).'.* ww(:).'.*( sum((xtune).^2,1) ))*scale;

% cost = 20*cost + 0.7*sum(tc(:).'.* ww(:).'.*( sum((xtune).^2,1) ))*scale ...
%                 + 0.7*sum(ww(:).'.*( sum((Xc(7:12,:)*10).^2,1) ))*scale ...
%                 + 0.3*sum(ww(:).'.*( sum((utune).^2,1) ))*scale;
% cost = 20*cost + 5*0.7*sum(ww(:).'.*( sum((xtune).^2,1) ))*scale ...
%                 + 3*0.7*sum(ww(:).'.*( sum((Xc(7:12,:)*10).^2,1) ))*scale ...
%                 + 0.3*sum(ww(:).'.*( sum((utune).^2,1) ))*scale;
cost = 20*cost ...
                + 0.7*sum(ww(:).'.*( sum((Xc(7:12,:)*10).^2,1) ))*scale ...
                + 0.3*sum(ww(:).'.*( sum((utune).^2,1) ))*scale;
% cost = 20*cost + 0.7*sum(ww(:).'.*( sum((W*xtune).^2,1) ))*scale ...
%                 + 0.7*sum(ww(:).'.*( sum((W*Xc(7:12,:)*10).^2,1) ))*scale ...
%                 + 0.3*sum(ww(:).'.*( sum((utune).^2,1) ))*scale;
% cost = 20*cost + 0.7*sum(ww(:).'.*( sum((xtune).^2,1) ))*scale ...
%                 + 0.7*sum(ww(:).'.*( sum((Xc(7:12,:)).^2,1) ))*scale ...
%                 + 0.3*sum(ww(:).'.*( sum((utune).^2,1) ))*scale;

% % path cost
% Xc = reshape(Xc,nS,numel(Xc)/nS);
% Uc = reshape(Uc,nU,numel(Uc)/nU);
% % Ucd = (D*(Uc.')/scale).';
% um=bnds(1:6);%vm=bnds(7:12);
% uscale=diag(1./um)*Uc;
% %vscale=diag(1./vm)*Xc(7:12,:);
% uscale=Uc/100;
% vscale=Xc(7:12,:)*3;
% 
% weights = [0.5,0.25,0.5];
% weights = [0.5,0.5,0.5];
% 
% cost = weights(1)*cost + weights(2)*sum(tc(:).'.* ww(:).'.*(0.5*sum(uscale.^2,1)...
%    +weights(3)*sum((vscale.*uscale).^2,1) ) )*scale;
% cost = weights(1)*cost + weights(2)*sum(ww(:).'.*(0.5*sum(uscale.^2,1)...
%    +weights(3)*sum((vscale.*uscale).^2,1) ) )*scale;
% 
% % cost = 0.5*cost + 0.5*sum(tc(:).'.* ww(:).'.*( sum(Ucd.^2,1) ))*scale;





end

