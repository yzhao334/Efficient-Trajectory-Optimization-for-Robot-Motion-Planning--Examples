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

function hd = plot_m16_3D_customR( time, theta, DH, tool, parts, viewang, framedis, frameflg,firsttime,prob )
% tempTool=tool*[...
%     eye(3),...
%     [0.006,-0.01,-0.825].';...
%     0 0 0 1
%     ];
tempTool=tool;
totLen = size(theta,1);
% default workspace limit
% xlimit=[-1,2];
% ylimit=[-0.4,1];
% zlimit=[0,2];
xlimit=prob.wall.x;
ylimit=prob.wall.y;
zlimit=prob.wall.z;
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
    set(hd(j),'edgecolor','none');
end

% hobs=patch('faces',prob.obs.faces,'vertices',prob.obs.vertices,...
%     'facecolor',[0,0.45,0.74],'facealpha',0.8);

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
% set(gca,'clipping','off');

if firsttime
    lighting flat;
    camlight right;
end

dt=mean(diff(time));
if frameflg
    timecost=0.2;
else
    timecost=0.025;
end

ratio=timecost/dt;
if ratio>1
    framedis=round(ratio);
else
    framedis=1;
end
if framedis==1
    tdelay=(dt-timecost)*0.7;
end

for i=1:framedis:totLen
    % forward kinematics
    [jntT]=kfwd_rob_m16_full(theta(i,:),DH,tool);
    for j=1:partsnum
        set(hd(j),'vertices',bsxfun(@plus,parts(j).data.vertices*jntT(1:3,1:3,parts(j).id).',jntT(1:3,4,parts(j).id).'));
    end
    drawnow;
    if frameflg
        [R,P,~]=kfwd_rob_m16(theta(i,:),DH,tempTool);
        frameplot(R,P,length);
        %hf = setframeplot(R,P,length,hf);
    end
    if framedis==1
        pause(tdelay);
    end
end


end

