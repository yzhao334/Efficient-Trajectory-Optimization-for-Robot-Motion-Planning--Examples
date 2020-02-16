function [c,r,obs,selfmap,wallmap,obsmap] = boundBall(prob)
% function [c,r,selfmap,wallmap,obs,obsmap] = boundBall(prob)
c=prob.rob.bnd_ball.c;
r=prob.rob.bnd_ball.r;
%% construct selcollisionfmap
s=1;
%1-5
for i=1:size(r{5},2)
    selfmap(s).l=[1,1]; selfmap(s).r=[5,i];
    s=s+1;
end
%2-5
for i=1:size(r{2},2)
    for j=1:size(r{5},2)
        selfmap(s).l=[2,i]; selfmap(s).r=[5,j];
        s=s+1;
    end
end
%% construct wall collision map
s=1;
% 4-wall
for i=1:length(r{4})
    wallmap(s).ind=[4,i];
    s=s+1;
end
% 5-wall
for i=1:length(r{5})
    wallmap(s).ind=[5,i];
    s=s+1;
end
%% obstacle ball approximation (single ball in the example)
obs.c=[0.8,-0.5,0.7;...
       0.5,0.7,0.3;...
       -0.6,0.5,0.3;...
       0.4,-0.5,1.6;...
       0.85,0,2.12];
obs.r=[0.4,0.4,0.3,0.3,0.3];

%% construct obsmap
s=1;
% 3-obs
for i=1:size(r{3},2)
    for j=1:size(obs.r,2)
        obsmap(s).l=[3,i]; obsmap(s).r=j;
        s=s+1;
    end
end
% 4-obs
for i=1:size(r{4},2)
    for j=1:size(obs.r,2)
        obsmap(s).l=[4,i]; obsmap(s).r=j;
        s=s+1;
    end
end
% 5-obs
for i=1:size(r{5},2)
    for j=1:size(obs.r,2)
        obsmap(s).l=[5,i]; obsmap(s).r=j;
        s=s+1;
    end
end

end