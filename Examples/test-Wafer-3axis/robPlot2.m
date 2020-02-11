function [ hret ] = robPlot2( q,Rob,bound,freq,pltBall )
plot_arc_resl=pi/20;
% clf;
hold on;
% set figure layout
grid on;axis equal;
xlim([-600,1100]);
% ylim([-700,900]);
ylim([-400,600])
% box on;
% xlabel('x (mm)');
% ylabel('y (mm)');
set(gcf,'color','w');
%construct a boundary
boundary_x0=[bound.corner.left_1(1),bound.corner.left_2(1),bound.corner.left_3(1),...
    bound.corner.left_4(1),bound.corner.right_4(1),bound.corner.right_3(1),bound.corner.right_2(1),...
    bound.corner.right_1(1)];
boundary_y0=[bound.corner.left_1(2),bound.corner.left_2(2),bound.corner.left_3(2),...
    bound.corner.left_4(2),bound.corner.right_4(2),bound.corner.right_3(2),bound.corner.right_2(2),...
    bound.corner.right_1(2)];
boundary_x0=[boundary_x0,boundary_x0(1)];
boundary_y0=[boundary_y0,boundary_y0(1)];
plot(boundary_x0,boundary_y0,'color','k','linewidth',2);
% construct graphic object for robot body, initial position
T1=trans(q(1,1)+Rob.joffset(1),Rob.l1,0);
T2=trans(q(1,2)+Rob.joffset(2),Rob.l2,0);
T3=trans(q(1,3)+Rob.joffset(3),Rob.l3,0);

elbow_x0=[Rob.elbow.r1*cos(pi+Rob.elbow.theta_1:plot_arc_resl:2*pi-Rob.elbow.theta_1),Rob.elbow.width,...
    Rob.elbow.width,Rob.elbow.r1*cos(Rob.elbow.theta_1:plot_arc_resl:pi-Rob.elbow.theta_1),-Rob.elbow.width,...
    -Rob.elbow.width];
elbow_x0=[elbow_x0,elbow_x0(1)];
elbow_y0=[Rob.elbow.r1*sin(pi+Rob.elbow.theta_1:plot_arc_resl:2*pi-Rob.elbow.theta_1),Rob.elbow.r,...
    Rob.elbow.r+Rob.elbow.sidelength,2*Rob.elbow.r+Rob.elbow.sidelength+Rob.elbow.r1*sin(Rob.elbow.theta_1:plot_arc_resl:pi-Rob.elbow.theta_1),Rob.elbow.r+Rob.elbow.sidelength,...
    Rob.elbow.r];
elbow_y0=[elbow_y0,elbow_y0(1)];
elbowPts=trans(pi/2,Rob.l1,0)\[elbow_x0;elbow_y0;ones(1,length(elbow_x0))];
%
motor_elbow_1_xy0=[0.6*Rob.elbow.r1*cos(0:plot_arc_resl:(2*pi));0.6*Rob.elbow.r1*sin(0:plot_arc_resl:(2*pi))];
motor_elbow_2_xy0=[0.6*Rob.elbow.r1*cos(0:plot_arc_resl:(2*pi));Rob.l1+0.6*Rob.elbow.r1*sin(0:plot_arc_resl:(2*pi))];
%
elbowM1Pts=trans(pi/2,Rob.l1,0)\[motor_elbow_1_xy0;ones(1,size(motor_elbow_1_xy0,2))];
elbowM2Pts=trans(pi/2,Rob.l1,0)\[motor_elbow_2_xy0;ones(1,size(motor_elbow_2_xy0,2))];

temp1=T1*elbowPts;
temp2=T1*elbowM1Pts;
temp3=T1*elbowM2Pts;
elbow= plot(temp1(1,:),temp1(2,:),'color',[0.5 0.5 0.5],'linewidth',2);
motor_elbow_1=plot(temp2(1,:),temp2(2,:),'color',[0.5 0.5 0.5]','linewidth',2);
motor_elbow_2=plot(temp3(1,:),temp3(2,:),'color','c','linewidth',2);

Rob.wrist.theta_1=pi/2-Rob.wrist.theta;
wrist_x0=[Rob.wrist.r2*cos(pi+Rob.wrist.theta_1:plot_arc_resl:2*pi-Rob.wrist.theta_1),Rob.wrist.width,...
   Rob.wrist.r1*cos(0:plot_arc_resl:pi),-Rob.wrist.width];
wrist_x0=[wrist_x0,wrist_x0(1)];
wrist_y0=[-(Rob.l2-Rob.l1)+Rob.wrist.r2*sin(pi+Rob.wrist.theta_1:plot_arc_resl:2*pi-Rob.wrist.theta_1),-(Rob.l2-Rob.l1-Rob.wrist.r),...
    Rob.l1+Rob.wrist.r1*sin(0:plot_arc_resl:pi),-(Rob.l2-Rob.l1-Rob.wrist.r)];
wrist_y0=[wrist_y0,wrist_y0(1)];
%
wristPts=(trans(pi/2,Rob.l1,0)*trans(pi,Rob.l2,0))\[wrist_x0;wrist_y0;ones(1,length(wrist_x0))];
%
motor_wrist_xy0=[0.6*Rob.wrist.r2*cos(0:plot_arc_resl:(2*pi));-(Rob.l2-Rob.l1)+0.6*Rob.wrist.r2*sin(0:plot_arc_resl:(2*pi))];
wristMPts=(trans(pi/2,Rob.l1,0)*trans(pi,Rob.l2,0))\[motor_wrist_xy0;ones(1,length(motor_wrist_xy0(1,:)))];
temp1=T1*T2*wristPts;% link points of initial position, link 2
temp2=T1*T2*wristMPts;
wrist= plot(temp1(1,:),temp1(2,:),'color','c','linewidth',2);
motor_wrist=plot(temp2(1,:),temp2(2,:),'color','r','linewidth',2);

Rob.wafer.theta=asin(Rob.wafer.width/Rob.wafer.r);
g03=trans(pi/2,Rob.l1,0)*trans(pi,Rob.l2,0)*trans(pi,Rob.l3,0)*trans(0,Rob.l2-Rob.l1,0);
wafer_x0=[Rob.wafer.r*cos(3*pi/2-Rob.wafer.theta:-plot_arc_resl:-pi/2+Rob.wafer.theta),Rob.wafer.width,...
    Rob.wafer.r1*cos(0:-plot_arc_resl:-pi)];
wafer_x0=[wafer_x0,wafer_x0(1)];
wafer_y0=[Rob.wafer.sidelength+Rob.wafer.r+Rob.wafer.r*sin(1.5*pi-Rob.wafer.theta:-plot_arc_resl:-pi/2+Rob.wafer.theta),0,Rob.wafer.r1*sin(0:-plot_arc_resl:-pi)];
wafer_y0=[wafer_y0,wafer_y0(1)];
%
waferPts=g03\[wafer_x0;wafer_y0;ones(1,length(wafer_x0))];

temp=T1*T2*T3*waferPts;
wafer= plot(temp(1,:),temp(2,:),'color','r','linewidth',2);
hret=[elbow,motor_elbow_1,motor_elbow_2,wrist,motor_wrist,wafer];
% bounding balls
t=linspace(0,2*pi);
len=[Rob.l1,Rob.l2,Rob.l3];
if pltBall
T(:,:,1)=T1;T(:,:,2)=T1*T2;T(:,:,3)=T1*T2*T3;
    for i=1:length(Rob.link)
        for j=1:length(Rob.link(i).c)
            ballRbt(i).pts(:,:,j)=...
                [Rob.link(i).c(j)-len(i)+Rob.link(i).r(j)*cos(t);...
                 Rob.link(i).r(j)*sin(t);...
                 ones(1,length(t))];
             temp=T(:,:,i)*ballRbt(i).pts(:,:,j);
                hbr(i).h(j)=plot(temp(1,:),temp(2,:),...
                    'color','k','linewidth',2);
        end
    end
    for i=1:length(Rob.obs.r)
        hobs(i)=plot(Rob.obs.cx(i)+Rob.obs.r(i)*cos(t),...
            Rob.obs.cy(i)+Rob.obs.r(i)*sin(t),...
            'color','k','linewidth',2);
    end
end
%% plot animation
clear temp;
for k=2:size(q,1)
    T1=trans(q(k,1)+Rob.joffset(1),Rob.l1,0);
    T2=trans(q(k,2)+Rob.joffset(2),Rob.l2,0);
    T3=trans(q(k,3)+Rob.joffset(3),Rob.l3,0);
    temp{1}=T1*elbowPts;
    temp{2}=T1*elbowM1Pts;
    temp{3}=T1*elbowM2Pts;
    temp{4}=T1*T2*wristPts;
    temp{5}=T1*T2*wristMPts;
    temp{6}=T1*T2*T3*waferPts;
    for j=1:length(hret)
        set(hret(j),'xdata',temp{j}(1,:),'ydata',temp{j}(2,:));
    end
    if pltBall
        T(:,:,1)=T1;T(:,:,2)=T1*T2;T(:,:,3)=T1*T2*T3;
        for i=1:length(Rob.link)
            for j=1:length(Rob.link(i).c)
                ballRbt(i).pts(:,:,j)=...
                    [Rob.link(i).c(j)-len(i)+Rob.link(i).r(j)*cos(t);...
                     Rob.link(i).r(j)*sin(t);...
                     ones(1,length(t))];
                 tempb=T(:,:,i)*ballRbt(i).pts(:,:,j);
                 set(hbr(i).h(j),'xdata',tempb(1,:),'ydata',tempb(2,:));
            end
        end
    end
    pause(1/freq);
end

end
