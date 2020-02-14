%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            3D Plot configuration of M16iB robot            %
%                   Yu Zhao, 2016/11/03                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%   Take one group of theta one time

% Inputs:   theta       joint position in rad unit (every dt one sample)
%           DH          DH parameter of robot
%           tool        tool frame relative to link 6 frame
%           parts       CAD model of m16 robot
%           viewang     [az,el], view angles
%           framedis    play frame 1:framedis:end, recommand 30 (dt 1ms)
%           frameflg    flag indicates whether plot coordinate frame
%           firsttime   flag indicates whether first time

% if plot frame, use 100 for framedis, otherwise 30

function hd = plot_m16_3D_customFrame( theta, DH, tool, parts, viewang, framedis, frameflg,firsttime,prob,showball )
% tempTool=tool*[...
%     eye(3),...
%     [0.006,-0.01,-0.825].';...
%     0 0 0 1
%     ];
tempTool=tool;
totLen = size(theta,1);
% default workspace limit
% xlimit=[-1,2];
% ylimit=[-0.5,1.5];
% zlimit=[0,2.5];
xlimit=prob.wall.x;
ylimit=prob.wall.y;
zlimit=prob.wall.z;
% xlimit=[-0.5,1.2];
% ylimit=[-0.5,0.5];
% zlimit=[0,1.6];
% recommanded view angle
% az=35;el=26;
% if viewang==0
%     viewang=[35,26];
% end
% default frame length
% length = 0.3;
length = 0.1;
% init plot
% clf;
[jntT]=kfwd_rob_m16_full(theta(1,:),DH,tool);
hold on;
set(gca,'Position',[0,0,1,1])
partsnum=numel(parts)-1;
hd=nan(partsnum,1);
for j=1:partsnum
    hd(j)=patch('faces',parts(j).data.faces,...
        'vertices',bsxfun(@plus,parts(j).data.vertices*jntT(1:3,1:3,parts(j).id).',jntT(1:3,4,parts(j).id).'));
    set(hd(j),'facecolor',parts(j).color);
%     set(hd(j),'edgecolor','none');
end

% hobs=patch('faces',prob.obs.faces,'vertices',prob.obs.vertices,...
%     'facecolor',[0,0.45,0.74],'facealpha',0.8);

if showball
    hball={};
    [xs,ys,zs]=sphere(20);
    for i=1:8
        for j=1:size(prob.br{i},2)
            temp=jntT(:,:,i)*[prob.bc{i}(j,:).';1];
            hball{i}(j)=surf(xs*prob.br{i}(j)+temp(1),...
                ys*prob.br{i}(j)+temp(2),...
                zs*prob.br{i}(j)+temp(3),...
                    'facecolor','c','facealpha',0.2);
        end
    end
    clear temp;
    hobsB=nan(1,size(prob.obs.r,2));
    for i=1:size(prob.obs.r,2)
        hobsB(i)=surf(xs*prob.obs.r(i)+prob.obs.c(i,1),...
                      ys*prob.obs.r(i)+prob.obs.c(i,2),...
                      zs*prob.obs.r(i)+prob.obs.c(i,3),'facecolor','k','facealpha',0.2);
    end
end

% if frameflg
%     [R,P,~]=kfwd_rob_m16(theta(1,:),DH,tempTool);
%     hf = frameplot(R,P,length);
% else
%     hf = nan(3,1);
% end

axis equal;
view(viewang);
xlim(xlimit);ylim(ylimit);zlim(zlimit);
axis vis3d;
box on;grid on;
% set(gca,'boxstyle','full');
% set(gca,'Clipping','off');

if firsttime
    lighting flat;
    camlight right;
end

for i=1:framedis:totLen
    % forward kinematics
    [jntT]=kfwd_rob_m16_full(theta(i,:),DH,tool);
    % plot frame
    if frameflg
        [R,P,~]=kfwd_rob_m16(theta(i,:),DH,tempTool);
        frameplot(R,P,length);
        %hf = setframeplot(R,P,length,hf);
    end
    % plot ball
    if showball
        for k=1:8
            for j=1:size(prob.br{k},2)
                temp=jntT(:,:,k)*[prob.bc{k}(j,:).';1];
                set(hball{k}(j),'xdata',xs*prob.br{k}(j)+temp(1),...
                    'ydata',ys*prob.br{k}(j)+temp(2),...
                    'zdata',zs*prob.br{k}(j)+temp(3)...
                    );
            end
        end
    end
    % plot part
    for j=1:partsnum
        set(hd(j),'vertices',bsxfun(@plus,parts(j).data.vertices*jntT(1:3,1:3,parts(j).id).',jntT(1:3,4,parts(j).id).'));
    end
    drawnow;
end


end

