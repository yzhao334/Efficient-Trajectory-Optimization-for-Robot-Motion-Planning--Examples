function plotRob( theta0, theta1, DH, tool, parts, viewang, framedis, frameflg,firsttime )
tempTool=tool;%*[...
%     eye(3),...
%     [0.006,-0.01,-0.825].';...
%     0 0 0 1
%     ];
totLen = size(theta0,1);
totLen1 = size(theta1,1);
% default workspace limit
% xlimit=[-1,2];
% ylimit=[-0.6,1];
% zlimit=[0,2];
xlimit=[-1,1.2];
ylimit=[-0.5,1.2];
zlimit=[0,1.6];
% recommanded view angle
% az=35;el=26;
if viewang==0
    viewang=[35,26];
end
% default frame length
% length = 0.3;
length = 0.1;
% init plot
% clf;
[jntT]=kfwd_rob_m16_full(theta0(1,:),DH,tool);
hold on;
xlabel('X [m]','fontsize',14);
ylabel('Y [m]','fontsize',14);
zlabel('Z [m]','fontsize',14);
partsnum=numel(parts)-1;
hd=nan(partsnum,1);
hd1=nan(partsnum,1);
for j=1:partsnum
    hd(j)=patch('faces',parts(j).data.faces,...
        'vertices',bsxfun(@plus,parts(j).data.vertices*jntT(1:3,1:3,parts(j).id).',jntT(1:3,4,parts(j).id).'));
    set(hd(j),'facecolor',parts(j).color);
    set(hd(j),'edgecolor','none');
    set(hd(j),'facealpha',0.2);
    hd1(j)=patch('faces',parts(j).data.faces,...
        'vertices',bsxfun(@plus,parts(j).data.vertices*jntT(1:3,1:3,parts(j).id).',jntT(1:3,4,parts(j).id).'));
    set(hd1(j),'facecolor',parts(j).color);
    set(hd1(j),'edgecolor','none');
end

axis equal;
view(viewang);
xlim(xlimit);ylim(ylimit);zlim(zlimit);
axis vis3d;
box on;grid on;
% set(gca,'boxstyle','full');

if firsttime
    lighting flat;
    camlight right;
end

for i=1:framedis:totLen
    % forward kinematics
    [jntT]=kfwd_rob_m16_full(theta0(i,:),DH,tool);
    if i<=totLen1
        [jntT1]=kfwd_rob_m16_full(theta1(i,:),DH,tool);
    end
    for j=1:partsnum
        set(hd(j),'vertices',bsxfun(@plus,parts(j).data.vertices*jntT(1:3,1:3,parts(j).id).',jntT(1:3,4,parts(j).id).'));
        if i<=totLen1
            set(hd1(j),'vertices',bsxfun(@plus,parts(j).data.vertices*jntT1(1:3,1:3,parts(j).id).',jntT1(1:3,4,parts(j).id).'));
        end
    end
    
    drawnow;
    if frameflg
%         [R,P,~]=kfwd_rob_m16(theta0(i,:),DH,tempTool);
%         frameplot(R,P,length);
        if i<=totLen1
            [R1,P1,~]=kfwd_rob_m16(theta1(i,:),DH,tempTool);
            %frameplot(R1,P1,length);
        end
        frameplot(R1,P1,length);
        %hf = setframeplot(R,P,length,hf);
    end
end


end

