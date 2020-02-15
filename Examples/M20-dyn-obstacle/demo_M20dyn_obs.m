% This is a toy example of optimal control based trajectory planning.
%   In this example, 3d 6-joint robot dynamics is considered, with
%   torque as control. Planning result is optimal motion and control for
%   time-optimally reaching target position under position, jerk, velocity,
%   acceleration, torque, and torque rate bound.
%   Self collision and workspace boundary is considered
% 
% After initialization, fast optimization can be performed when target
% changed.

%% clean up
clear all;clc;
%% add dependends
addpath('../PseudoOpt');
addpath('../misc');
addpath(genpath('../Utils'));
%% set up parameter
prob.rob = RobABA('M16iB');
prob.si.m16.Tool = prob.rob.rtb.tool;
prob.parts = load('M16CAD','parts');
prob.parts = prob.parts.parts;%loadCAD_m16(prob.si);
prob.si.DH(:,1) = prob.rob.r.Alpha(:);
prob.si.DH(:,2) = prob.rob.r.A(:);
prob.si.DH(:,3) = prob.rob.r.D(:);
prob.si.DH(:,4) = prob.rob.r.Theta0(:);
[prob.bc,prob.br,prob.wall,...
    prob.selfmap,prob.wallmap,prob.obs,prob.obsmap]=boundBall();
%% initialize optimization
ps=PseudoOptimal;
ps.npts=8;%12;
ps.nS=12;
ps.nU=6;
maxiter=10000;disp=5;
ps=ps.timeOptimalInit(@(Xc,Uc,D,nS,nU,scale,P)conEq(Xc,Uc,D,nS,nU,scale,P,prob.rob),...
    @(Xc,Uc,tc,D,nS,nU,ww,scale)costFcn(Xc,Uc,tc,D,nS,nU,ww,scale,prob.rob),...
    @(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale,prob),maxiter,disp);
%% set up and solve optimal control problem
tf=10;dt=0.01;
%% target 1 + plot
R=[0 0 1;0  1 0;-1 0 0]; % #1, #3
P=[-0.5;0.8;0.4]; % #1, #2, #4
init=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
R=[0 0 1;0  1 0;-1 0 0];
P=[1.5;0;0.33]; % #1
target=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
[opt] = OptTrj(prob,ps,init,target,tf,dt);
clf;hrob=plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts,[134,20], 10, 0 , 1 , prob);%[150,30]
%% target 2 + plot
R=[0 0 1;0  1 0;-1 0 0]; % #1, #3
R=R*rotz(-pi/6); % #2, #4
P=[-0.5;0.8;0.4]; % #1, #2, #4
init=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
R=[0 0 1;0  1 0;-1 0 0];
P=[0.75;0.5;0.33]; % #2
target=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
[opt] = OptTrj(prob,ps,init,target,tf,dt);
clf;hrob=plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts,[134,20], 10, 0 , 1 , prob);%[150,30]
%% target 2inverse + plot
R=[0 0 1;0  1 0;-1 0 0];
P=[0.75;0.5;0.33]; % #2
init=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
R=[0 0 1;0  1 0;-1 0 0]; % #1, #3
R=R*rotz(-pi/6); % #2, #4
P=[-0.5;0.8;0.4]; % #1, #2, #4
target=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
[opt] = OptTrj(prob,ps,init,target,tf,dt);
clf;hrob=plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts,[134,20], 10, 0 , 1 , prob);%[150,30]
%% target 3 + plot
R=[0 0 1;0  1 0;-1 0 0]; % #1, #3
P=[1;0.8;0.33]; % #3
init=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
R=[0 0 1;0  1 0;-1 0 0];
P=[0.25;0.6;0.85]; % #3
target=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
% [opt] = OptTrj(prob,ps,init,target,tf,dt);
[opt] = OptTrj_NI(prob,ps,init,target,tf,dt);
clf;hrob=plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts,[134,20], 10, 0 , 1 , prob);%[150,30]
%% target 4 + plot
R=[0 0 1;0  1 0;-1 0 0]; % #1, #3
R=R*rotz(-pi/6); % #2, #4
P=[-0.5;0.8;0.4]; % #1, #2, #4
init=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
R=[0 0 1;0  1 0;-1 0 0];
P=[0.7;0.7;0.65]; % #4
target=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);
% [opt] = OptTrj(prob,ps,init,target,tf,dt);
[opt] = OptTrj_NI(prob,ps,init,target,tf,dt);
clf;hrob=plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts,[134,20], 10, 0 , 1 , prob);%[150,30]
%% plot bnd
velbnd=pi/180*[165 165 175 350 340 520];
trqbnd=[1396.5,1402.3,382.7,45.2,44.6,32.5].'*0.8;% reduced to 80%
dtrqbnd=15*trqbnd;
for i=1:6
    accopt(:,i)=gradient(opt.Xopt(:,6+i))./gradient(opt.Topt(:));
end
trqm=opt.Uopt+accopt*diag(prob.rob.r.Jm.*prob.rob.r.G.^2);
for i=1:6
    dtrq(:,i)=gradient(trqm(:,i))./gradient(opt.Topt(:));
end
% %% test case
% % init=[-pi/3 pi*4/18 -pi*4/18 0 0 pi/4];
% % target=[0 -pi/6 pi/6 0 0 pi/2];
% init=[-pi*4/18 pi*4.5/18 -pi*4.5/18 0 0 pi/2];
% target=[0 -pi/3 pi/3 0 0 pi/2];
% [opt,ps] = OptTrj(prob,ps,init,target,tf,dt);
% % clf;plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts, [90,0], 10, 0 , 1 , prob);
% clf;plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts, [150,30], 10, 0 , 1 , prob);
% % clf;
% % plot_m16_3D_customFrame( opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts, [90,0], 30, 1 , 0,prob,0 );
%%
time0=0:0.001:opt.Topt(end);
time0=time0(:);
for i=1:12
    temppos(:,i)=interp1(opt.Topt,opt.Xopt(:,i),time0);
end
for i=1:6
    tempu(:,i)=interp1(opt.Topt,opt.Uopt(:,i),time0);
end
opt.Xopt=temppos;
opt.Uopt=tempu;
opt.Topt=time0;