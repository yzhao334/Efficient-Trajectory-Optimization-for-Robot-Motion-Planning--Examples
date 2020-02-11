function [ hret ] = robPlot( pos,rob,freq )
% plot robot configuration for 2d 2-link robot
% author: Yu Zhao, yzhao334@berkeley.edu
% 
% INPUT:
%   pos = [2, n] = [q1;q2] = position sequences
%   rob = robot structure
%   freq = plot fresh frequency
% OUTPUT:
%   hret: includes h1,h2,hbr1,hbr2
%       h1: handle of line for link 1
%       h2: handle of line for link 2
%       hbr1: handle series of ball on link 1
%       hbr2: handle series of ball on link 2

% transform matrices function
trans = @(q,x,y)[[cos(q) sin(q);-sin(q) cos(q)].',[x;y];0,0,1];
% initialize plot
clf;hold on;
% set figure layout
grid on;axis equal;
xlim([-2,2]);ylim([-2,2]);
box on;
% source points for links, [x;y;1]
l1pts=[0,rob.kin.l1;0,0;1,1];
l2pts=[0,rob.kin.l2;0,0;1,1];
distlev=20;
cirpts=[rob.kin.br*cos(linspace(0,pi*2,distlev));
        rob.kin.br*sin(linspace(0,pi*2,distlev));
        ones(1,distlev)];
% plot initial position
T1=trans(pos(1,1),0,0);
T2=trans(pos(2,1),rob.kin.l1,0);
temp1=T1*l1pts;% link points of initial position, link 1
temp2=T1*T2*l2pts;% link points of initial position, link 2

h1 = plot(temp1(1,:),temp1(2,:),'b','linewidth',3);
h2 = plot(temp2(1,:),temp2(2,:),'g','linewidth',3);
hbr1=nan(1,length(rob.kin.l1pts));
hbr2=nan(1,length(rob.kin.l2pts));
for k=1:length(rob.kin.l1pts)
    temp=T1*(cirpts+[rob.kin.l1pts(k);0;0]);
    hbr1(k) = plot(temp(1,:),temp(2,:),'r','linewidth',2);
end
for k=1:length(rob.kin.l2pts)
    temp=T1*T2*(cirpts+[rob.kin.l2pts(k);0;0]);
    hbr2(k) = plot(temp(1,:),temp(2,:),'r','linewidth',2);
end
% plot animation
for k=2:size(pos,2)
    T1=trans(pos(1,k),0,0);
    T2=trans(pos(2,k),rob.kin.l1,0);
    temp1=T1*l1pts;
    temp2=T1*T2*l2pts;
    set(h1,'xdata',temp1(1,:),'ydata',temp1(2,:));
    for j=1:length(rob.kin.l1pts)
        temp=T1*(cirpts+[rob.kin.l1pts(j);0;0]);
        set(hbr1(j),'xdata',temp(1,:),'ydata',temp(2,:));
    end
    set(h2,'xdata',temp2(1,:),'ydata',temp2(2,:));
    for j=1:length(rob.kin.l2pts)
        temp=T1*T2*(cirpts+[rob.kin.l2pts(j);0;0]);
        set(hbr2(j),'xdata',temp(1,:),'ydata',temp(2,:));
    end
    pause(1/freq);
end
% plot handles
hret.h1 = h1;
hret.h2 = h2;
hret.hbr1 = hbr1;
hret.hbr2 = hbr2;

end
