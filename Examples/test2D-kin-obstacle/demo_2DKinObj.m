% This is a toy example of optimal control based trajectory planning.
%   In this example, 2d 2-link robot kinematics is used as 'dynamics', with
%   jerk as control. Planning result is optimal motion and control for
%   time-optimally reaching target position under jerk, velocity, and
%   acceleration bound. Obstacle is also considered.
% 
% author: Yu Zhao, yzhao334@berkeley.edu

%% set up parameter
prob.rob = rob_params;
prob.time = linspace(0,9.9,100);
%% set up optimal control problem
prob.init = [0;-pi/4];
prob.target = [pi/2;0];% joint space target position
ps=PseudoOptimal;
ps.npts=20;
ps.nS=6;
ps.nU=2;
ps.sGuess=interp1([0;10],[prob.init(1),prob.init(2),0,0,0,0;prob.target(1),prob.target(2),0,0,0,0],prob.time(:));
prob.u_init=zeros(length(prob.time),2);
prob.init=ps.sGuess(1,:).';
[prob.lb,prob.ub]=bounds(ps.npts,prob.rob);
%% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimal(@conEq,@costFcn,@(Xc,Uc,D,scale)conIneq(Xc,Uc,D,scale,prob.target,prob.rob),...
    prob.lb,prob.ub,prob.time,prob.u_init,...
    prob.init,1000,5);
%% plot
% animation
h=robPlot(opt.Xopt(:,1:2).',prob.rob,36);
% plot result
figure;
subplot(3,1,1);
plot(opt.Topt,opt.Xopt(:,1:2),'linewidth',2);
grid on;xlim([opt.Topt(1),opt.Topt(end)]);
legend('jnt1 pos','jnt2 pos');
xlabel('time [s]');ylabel('pos [rad]');
subplot(3,1,2);
plot(opt.Topt,opt.Xopt(:,3:4),'linewidth',2);
ylim([-2.1,2.1]);
grid on;xlim([opt.Topt(1),opt.Topt(end)]);
legend('jnt1 vel','jnt2 vel');
xlabel('time [s]');ylabel('vel [rad/s]');
subplot(3,1,3);
plot(opt.Topt,opt.Xopt(:,5:6),'linewidth',2);
ylim([-5.1,5.1]);
grid on;xlim([opt.Topt(1),opt.Topt(end)]);
legend('jnt1 acc','jnt2 acc');
xlabel('time [s]');ylabel('acc [rad/s^2]');

