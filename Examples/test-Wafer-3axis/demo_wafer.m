%Time optimal control based trajectory planning for wafer handling robot
% 2D wafer handling robot, obscatle avoidance with kinematics
% 
% constraints include robot joint kinematic constraints, workspace limit
% (obstacle avoidance), workspace acceleration constraints to prevent
% dropping wafer.
% 
% constraints, description refer to "Trajectory planning for robot 
% manipulators considering kinematic constraints using probabilistic 
% roadmap approach." Xiaowen Yu etc., 2017, or "Intelligent Control and 
% Planning for Industrial Robots." Yu Zhao, 2018.
% 
% author: Yu Zhao, yzhao334@berkeley.edu

%% set up parameter
[prob.rob,prob.bnd] = RobParam;
prob.tf0=10;
prob.time = linspace(0,prob.tf0,100);
%% set up optimal control problem
% T=genT(-80,[-252.5,-47.7,30]);
T=genT(-80,[-252.5,-35,30]);
% T=genT(-90,[-252.5,-40,30]);
[~,temp1,temp2,temp3]=inverseKine(T,-1);
prob.init = [temp1;temp2;temp3];
% T=genT(90-7.5,[-148.9,213.1,576]);
% [~,temp1,temp2,temp3]=inverseKine(T,-1);
T=genT(90,[700,213.1,576]);
[~,temp1,temp2,temp3]=inverseKine(T,1);
prob.target = [temp1;temp2;temp3];% joint space target position
ps=PseudoOptimal;
ps.npts=20;
ps.nS=9;
ps.nU=3;
ps.sGuess=interp1([0;prob.tf0],[prob.init(:).',zeros(1,6);prob.target(:).',zeros(1,6)],prob.time(:));
prob.u_init=zeros(length(prob.time),ps.nU);
prob.init=ps.sGuess(1,:).';
[prob.lb,prob.ub]=bounds(ps.npts,prob.rob,prob.bnd);
%% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimal(@conEq,@costFcn,@(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale,prob.target,prob.rob),...
    prob.lb,prob.ub,prob.time,prob.u_init,...
    prob.init,1000,4);
%% plot
% animation
robPlot(opt.Xopt(:,1:3),prob.rob,prob.bnd,30,0);
% plot result
figure;
subplot(3,1,1);
plot(opt.Topt,[opt.Xopt(:,4)/4.2,opt.Xopt(:,5)/8.4,opt.Xopt(:,6)/6.3],'linewidth',2);
grid on;xlim([opt.Topt(1),opt.Topt(end)]);
legend('jnt1 vel/Vm','jnt2 vel/Vm','jnt3 vel/Vm');
xlabel('time [s]');ylabel('vel ratio');
subplot(3,1,2);
plot(opt.Topt,[opt.Xopt(:,7)/8.4,opt.Xopt(:,8)/16.8,opt.Xopt(:,9)/12.6],'linewidth',2);
grid on;xlim([opt.Topt(1),opt.Topt(end)]);
legend('jnt1 acc/Am','jnt2 acc/Am','jnt3 acc/Am');
xlabel('time [s]');ylabel('acc ratio');
subplot(3,1,3);
acc=xyAcc(opt.Xopt(:,1:3).',opt.Xopt(:,4:6).',opt.Xopt(:,7:9).',prob.rob);
plot(opt.Topt,acc.','linewidth',2);
ylim([-980,980]);
grid on;xlim([opt.Topt(1),opt.Topt(end)]);
legend('x acc','y acc');
xlabel('time [s]');ylabel('tip acc [mm/s^2]');

