% This is a toy example of optimal control based trajectory planning.
%   In this example, 3d 6-joint robot dynamics is considered, with
%   torque as control. Planning result is optimal motion and control for
%   time-optimally reaching target position under position, jerk, velocity,
%   acceleration, torque, and torque rate bound.
%   Self collision is considered.
%   robot links is restricted to be above ground
% 
% After initialization, fast optimization can be performed when target
% changed.
% 
% author: Yu Zhao, yzhao334@berkeley.edu

%% set up parameter
prob.rob = RobViz('M20iA');
[prob.c,prob.r,prob.obs,...
    prob.selfmap,prob.wallmap,prob.obsmap] =...
    boundBall(prob);
%% initialize optimization
ps=PseudoOptimal;
ps.npts=12;
ps.nS=18;
ps.nU=6;
maxiter=500;dispLev=5;
switch regul
    case 'y'
        ps=ps.timeOptimalInit(@(Xc,Uc,D,nS,nU,scale,P)conEq(Xc,Uc,D,nS,nU,scale,P,prob.rob),...
        @(Xc,Uc,tc,D,nS,nU,ww,scale)costFcn(Xc,Uc,tc,D,nS,nU,ww,scale,prob.rob),...
        @(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale,prob),maxiter,dispLev);
    case 'n'
        ps=ps.timeOptimalInit(@(Xc,Uc,D,nS,nU,scale,P)conEq(Xc,Uc,D,nS,nU,scale,P,prob.rob),...
        @(Xc,Uc,tc,D,nS,nU,ww,scale)costFcn_topt(Xc,Uc,tc,D,nS,nU,ww,scale,prob.rob),...
        @(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale,prob),maxiter,dispLev);
    otherwise
        disp('Wrong option. Please try again');
        return;
end
switch showbndball
    case 'y'
        robinitplotplus=@()prob.rob.initBndBallPlot();
        robdrawfcn=@(q)prob.rob.drawWBnd(q);
    case 'n'
        robinitplotplus=@()[];
        robdrawfcn=@(q)prob.rob.draw(q);
    otherwise
        disp('Wrong option. Please try again');
        return;
end
disp('Initialization finished.');
disp('  ');
%% set up series of trials
% initial setup
prob.tf=0.96;
prob.dt=0.01;
prob.time = 0:prob.dt:prob.tf;
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,zeros(1,6),zeros(1,6),prob);
velbnd=prob.bnds(:,3);
trqbnd=prob.bnds(:,4);
dtrqbnd=prob.bnds(:,5);
rotx=@(x)[1,0,0;0,cos(x),sin(x);0,-sin(x),cos(x)].';
rotz=@(x)[cos(x),sin(x),0;-sin(x),cos(x),0;0,0,1].';
roty=@(x)[cos(x),0,-sin(x);0,1,0;sin(x),0,cos(x)].';
state_record=[];control_record=[];time_record=[];
%% task 1
prob.init = [0 0 0 0 0 0];% joint space initial position
prob.target = [0,0,-pi/2,0,0,0];% joint space target position
prob.targetPos = prob.rob.tcpPos(prob.target);
disp(['Initial pos [',num2str(prob.init(:).'),']']);
disp(['Target pos [',num2str(prob.targetPos(:).'),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:6
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    ps.sGuess(:,7+i) = acc(:);
    prob.u_init(:,i) = gradient(
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.targetPos,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.jnt_pos=prob.init;
prob.rob.InitPlot([150,30]);
robinitplotplus();
plotBalls(prob.obs.c,prob.obs.r,[0.7,0.7,0.7]);
pause(0.01);
for j=1:5:length(opt.Topt)
    robdrawfcn(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end
%% task 2
% pause(1);
prob.init=opt.Xopt(end,1:6);
prob.target=[pi/3,pi/3,-pi/4,0,-pi/3,0];% joint space target position
prob.targetPos = prob.rob.tcpPos(prob.target);
disp(['Initial pos [',num2str(prob.init(:).'),']']);
disp(['Target pos [',num2str(prob.targetPos(:).'),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.targetPos,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.jnt_pos=prob.init;
prob.rob.InitPlot([150,30]);
robinitplotplus();
hs=plotBalls(prob.obs.c,prob.obs.r,[0.7,0.7,0.7]);
pause(0.01);
for j=1:5:length(opt.Topt)
    robdrawfcn(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end
%% task 3
% pause(1);
R=[1 0 0;0 0 -1; 0 1 0]*rotx(pi/2)*rotz(-pi/6);
P=[0.5;0.2;1.3];
prob.init=opt.Xopt(end,1:6);
prob.target=prob.rob.kinv_M20iA(R,P);
prob.targetPos = prob.rob.tcpPos(prob.target);
disp(['Initial pos [',num2str(prob.init(:).'),']']);
disp(['Target pos [',num2str(prob.targetPos(:).'),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.targetPos,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.jnt_pos=prob.init;
prob.rob.InitPlot([150,30]);
robinitplotplus();
hs=plotBalls(prob.obs.c,prob.obs.r,[0.7,0.7,0.7]);
pause(0.01);
for j=1:5:length(opt.Topt)
    robdrawfcn(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end
%% task 4
% pause(1);
R=[0 1 0;0 0 -1;-1 0 0];
P=[-0.6;-0.4;0.4];
prob.init=opt.Xopt(end,1:6);
prob.target=prob.rob.kinv_M20iA(R,P);
prob.targetPos = prob.rob.tcpPos(prob.target);
disp(['Initial pos [',num2str(prob.init(:).'),']']);
disp(['Target pos [',num2str(prob.targetPos(:).'),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.targetPos,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.jnt_pos=prob.init;
prob.rob.InitPlot([150,30]);
robinitplotplus();
hs=plotBalls(prob.obs.c,prob.obs.r,[0.7,0.7,0.7]);
pause(0.01);
for j=1:5:length(opt.Topt)
    robdrawfcn(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end
%% task 5
% pause(1);
R=[1 0 0;0 0 -1; 0 1 0]*rotx(pi/2)*rotz(-pi/6);
P=[0.5;0.2;1.3];
prob.init=opt.Xopt(end,1:6);
prob.target=prob.rob.kinv_M20iA(R,P);
prob.targetPos = prob.rob.tcpPos(prob.target);
disp(['Initial pos [',num2str(prob.init(:).'),']']);
disp(['Target pos [',num2str(prob.targetPos(:).'),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.targetPos,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.jnt_pos=prob.init;
prob.rob.InitPlot([150,30]);
robinitplotplus();
hs=plotBalls(prob.obs.c,prob.obs.r,[0.7,0.7,0.7]);
pause(0.01);
for j=1:5:length(opt.Topt)
    robdrawfcn(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end
%% task 6
% pause(1);
R=[1 0 0;0 0 -1; 0 1 0]*rotx(pi/2)*rotz(-pi/6);
P=[0.1;0.7;1.7];
prob.init=opt.Xopt(end,1:6);
prob.target=prob.rob.kinv_M20iA(R,P);
prob.targetPos = prob.rob.tcpPos(prob.target);
disp(['Initial pos [',num2str(prob.init(:).'),']']);
disp(['Target pos [',num2str(prob.targetPos(:).'),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.targetPos,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.jnt_pos=prob.init;
prob.rob.InitPlot([150,30]);
robinitplotplus();
hs=plotBalls(prob.obs.c,prob.obs.r,[0.7,0.7,0.7]);
pause(0.01);
for j=1:5:length(opt.Topt)
    robdrawfcn(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end
%% task 7
% pause(1);
R=[1 0 0;0 0 -1; 0 1 0]*rotx(pi/2);
P=[1;0.3;0.6];
prob.init=opt.Xopt(end,1:6);
prob.target=prob.rob.kinv_M20iA(R,P);
prob.targetPos = prob.rob.tcpPos(prob.target);
disp(['Initial pos [',num2str(prob.init(:).'),']']);
disp(['Target pos [',num2str(prob.targetPos(:).'),']']);
tmp=input('Press any key to run ');
prob.tf=20*max(abs(prob.target(:)-prob.init(:))./velbnd(:));
prob.time = 0:prob.dt:prob.tf;
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
sAcc=nan(length(prob.time),ps.nS/2);
prob.u_init=nan(length(prob.time),ps.nU);
for i=1:ps.nS/2
    [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    sAcc(:,i) = acc(:);
end
for j=1:size(sAcc,1)
    prob.u_init(j,:)=prob.rob.rne_s(ps.sGuess(j,1:6),ps.sGuess(j,7:12),sAcc(j,:));
end
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.targetPos,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
% animation
figure(1);clf;
prob.rob.jnt_pos=prob.init;
prob.rob.InitPlot([150,30]);
robinitplotplus();
hs=plotBalls(prob.obs.c,prob.obs.r,[0.7,0.7,0.7]);
pause(0.01);
for j=1:5:length(opt.Topt)
    robdrawfcn(opt.Xopt(j,1:6));
    pause(0.02);
end
% record data
state_record = [state_record;opt.Xopt];
control_record = [control_record;opt.Uopt];
if isempty(time_record)
    time_record = [time_record;opt.Topt(:)];
else
    time_record = [time_record;opt.Topt(:)+time_record(end)+prob.dt];
end

%% plot recorded results
% plot result
for j=1:6
    ratioVel(:,j)=state_record(:,6+j)/velbnd(j);
    ratioTrq(:,j)=control_record(:,j)/trqbnd(j);
    dtrq(:,j)=gradient(control_record(:,j))./gradient(time_record(:));
    ratioDtrq(:,j)=dtrq(:,j)/dtrqbnd(j);
end
figure(2);
clf;
subplot(3,1,1);
plot(time_record,ratioVel,'linewidth',2);
grid on;xlim([time_record(1),time_record(end)]);
ylim([-1,1]);
title('Velocity/Vm');
xlabel('time [s]');ylabel('vel ratio');
subplot(3,1,2);
plot(time_record,ratioTrq,'linewidth',2);
grid on;xlim([time_record(1),time_record(end)]);
ylim([-1,1]);
title('Torque/Tm');
xlabel('time [s]');ylabel('trq ratio');
subplot(3,1,3);
plot(time_record,ratioDtrq,'linewidth',2);
ylim([-1,1]);
grid on;xlim([time_record(1),time_record(end)]);
title('Torque rate/dTm');
xlabel('time [s]');ylabel('trq rate ratio');
