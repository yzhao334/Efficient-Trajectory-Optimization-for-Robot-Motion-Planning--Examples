% This is a toy example of optimal control based trajectory planning.
%   In this example, 3d 6-joint robot dynamics is considered, with
%   torque as control. Planning result is optimal motion and control for
%   time-optimally reaching target position under position, jerk, velocity,
%   acceleration, torque, and torque rate bound.
% 
% After initialization, fast optimization can be performed when target
% changed. No obstacle or workspace boundary is considered
% 
% author: Yu Zhao, yzhao334@berkeley.edu

%% set up parameter
prob.rob = RobViz('M20iA');
% prob.parts = load('M16CAD','parts');
% prob.parts = prob.parts.parts;%loadCAD_m16(prob.si);
% prob.si.DH(:,1) = prob.rob.r.Alpha(:);
% prob.si.DH(:,2) = prob.rob.r.A(:);
% prob.si.DH(:,3) = prob.rob.r.D(:);
% prob.si.DH(:,4) = prob.rob.r.Theta0(:);
% [prob.bc,prob.br,prob.wall,...
%     prob.selfmap,prob.wallmap,prob.obs,prob.obsmap]=boundBall();
%% initialize optimization
ps=PseudoOptimal;
ps.npts=12;
ps.nS=12;
ps.nU=6;
maxiter=200;disp=5;
ps=ps.timeOptimalInit(@(Xc,Uc,D,nS,nU,scale,P)conEq(Xc,Uc,D,nS,nU,scale,P,prob.rob),...
    @(Xc,Uc,tc,D,nS,nU,ww,scale)costFcn(Xc,Uc,tc,D,nS,nU,ww,scale,prob.rob),...
    @(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale),maxiter,disp);
%% set up and solve optimal control problem
prob.init = [0 0 0 0 0 0];% joint space initial position
% prob.target = [pi/5,-pi/4,pi/4,-pi/10,-pi/4,pi/10];% joint space target position
% prob.target = [pi*2/3,-pi/4,pi/4,-pi/10,-pi/4,pi/10];% joint space target position
% prob.target = [pi/3,pi/8,-6/9/2*pi,-pi/10,-pi/4,pi/10];% infeasible target if unrelax j3
% prob.target = [pi/3,-pi/8,-pi/8,pi/10,-pi/10,0];% joint space target position
% prob.target = [0 0 0 pi/6 0 0];% joint space target position
prob.target = [pi/3,pi/3,-pi/4,0,-pi/3,0];% joint space target position
%%
% R=[0 0 1;0  1 0;-1 0 0];
R=[1 0 0;0 0 -1; 0 1 0]*rotx(pi/2)*rotz(-pi/6);
% P=[-0.5;0.8;0.4];
P=[0.5;0.2;1.3];
% % P=[-0.5;0.5;0.4];
% % P=[1;0.8;0.33];
prob.init=prob.rob.kinv_M20(R,P);
% % init=prob.rob.rtb.ikine([0 0 1 -0.5;0 1 0 0.95;-1 0 0 0.4;0 0 0 1],[pi*2/3 -pi/6 -pi/6 0 -pi/6 pi/6]);
% R=[0 1 0;0 0 -1;-1 0 0];
% % P=[1;0.8;0.33];
% % P=[1;0;0.33];
% P=[0.75;0.5;0.33];
% P=[0.25;0.6;0.85];
% % P=[0.5;0.6;0.85];
% % P=[0.7;0.7;0.65];
R=R*rotz(pi/6);
P=[1.2;0.5;1.3];
prob.target=kinv_rob_m16_modeR(R,P,prob.si.DH,prob.rob.rtb.tool);

%%
clf;
prob.tf=0.96;%10;
% prob.time=linspace(0,prob.tf);
% prob.dt=prob.tf/99;
prob.dt=0.01;
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
prob.u_init=prob.rob.rtb.rne(ps.sGuess(:,1:6),ps.sGuess(:,7:12),sAcc);
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.target,prob.rob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% plot
% plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts, [150,30], 10, 0 , 1 );
plot_m16_3D_customR( opt.Topt, opt.Xopt(:,1:6), prob.si.DH, prob.rob.rtb.tool, prob.parts, [150,30], 10, 0 , 1, prob );

velbnd=pi/180*[165 165 175 350 340 520].';
trqbnd=[1396.5,1402.3,382.7,45.2,44.6,32.5].'*0.8;% reduced to 80%
% dtrqbnd=15*trqbnd;
dtrqbnd=[4945,7769,1407,33.9,129,39].'; % task specific

accopt=[];
jerkopt=[];
dtrq=[];
for i=1:6
    accopt(:,i)=gradient(opt.Xopt(:,6+i))./gradient(opt.Topt(:));
    jerkopt(:,i)=gradient(accopt(:,i))./gradient(opt.Topt(:));
end
trqm=opt.Uopt+accopt*diag(prob.rob.r.Jm.*prob.rob.r.G.^2);
for i=1:6
    dtrq(:,i)=gradient(trqm(:,i))./gradient(opt.Topt(:));
end