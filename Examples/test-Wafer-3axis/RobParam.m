function [Rob,bound] = RobParam()
%% generating robot parameters
z=0;
%% parameters
% Robot link lengths
Rob.l1=360;
Rob.l2=440;
Rob.l3=350;
% Robot joint range
Rob.range.q1=[-170,170];
Rob.range.q2=[-170,170];
Rob.range.q3=[-190,190];
% elbow parameters
Rob.elbow.r1=60;%radius of the circle
Rob.elbow.r=65;% is the length of center to llpoint_x
Rob.elbow.width=120;%half of the width
Rob.elbow.l=sqrt(Rob.elbow.r^2+Rob.elbow.width^2);
% Rob.elbow.theta=0.4964+0.4552;
Rob.elbow.theta=asin(Rob.elbow.r1/Rob.elbow.l)+atan(Rob.elbow.r/Rob.elbow.width);%angle between top side length to the horizontal axis(x axis)
Rob.elbow.sidelength=Rob.l1-2*Rob.elbow.r;

%wrist parameters
Rob.wrist.r1=Rob.elbow.r1;%radius at the end that connects to elbow
Rob.wrist.r2=90/2;%radius at the end connects to blade
Rob.wrist.r=100;% from the circle center of end 2 to the corner point
Rob.wrist.width=Rob.wrist.r1;%half of the width
Rob.wrist.l=sqrt(Rob.wrist.r^2+Rob.wrist.width^2);
Rob.wrist.theta=asin(Rob.wrist.r2/Rob.wrist.l)+atan(Rob.wrist.r/Rob.wrist.width);%angle between end2 side length to the horizontal axis(x axis)
Rob.wrist.sidelength=Rob.l2-Rob.wrist.r;

%wafer parameter
Rob.wafer.r=150;% 300mm wafer
Rob.wafer.r1=90/2;% radius of the end connects to the wrist
Rob.wafer.sidelength=200;
Rob.wafer.width=90/2;% half of the link width

% boundary parameters
bound.corner.left_1=[-350,460,1]';
bound.corner.left_2=[-350,360,1]';
bound.corner.left_3=[-500,360,1]';
bound.corner.left_4=[-500,-200,1]';
bound.corner.right_1=[850,460,1]';
bound.corner.right_2=[850,360,1]';
bound.corner.right_3=[1000,360,1]';
bound.corner.right_4=[1000,-200,1]';
bound.vec.lu=[0,1,0]';
bound.vec.ll=[-1,0,0]';
bound.vec.ru=[0,1,0]';
bound.vec.rl=[1,0,0]';
bound.leftbound=-500;
bound.rightbound=1000;
bound.upperbound=460;
bound.lowerbound=-200;
bound.secondleftbound=-350;
bound.secondrightbound=850;
bound.secondupperbound=360;
% extra parameters
Rob.elbow.theta_1=pi/2-Rob.elbow.theta;
Rob.wafer.theta=asin(Rob.wafer.width/Rob.wafer.r);
Rob.wrist.theta_1=pi/2-Rob.wrist.theta;
Rob.joffset=[pi/2,pi,pi];

Rob.link(1).c=[100,300];
Rob.link(1).r=[125,125];
Rob.link(2).c=[110,110*2,110*3,440];
Rob.link(2).r=[70,70,70,50];
Rob.link(3).c=[125,Rob.l3];
Rob.link(3).r=[70,150];
Rob.obs.cx=[-400,-480,900,980];
Rob.obs.cy=[410,410,410,410];
Rob.obs.r=[60,60,60,60];
end