function [opt,ps] = OptTrj(prob,ps,init,target,tf,dt)
prob.init = init;% joint space initial position
prob.target = target;% joint space target position

prob.tf=tf;
prob.dt=dt;
prob.time = 0:prob.dt:prob.tf;
% add intelligent initialization
T0=prob.rob.rtb.fkine(init);
T2=prob.rob.rtb.fkine(target);
T1=T0;T1(1:3,4)=(T0(1:3,4)+T2(1:3,4))/2;T1(3,4)=1;
mid=kinv_rob_m16_modeR(T1(1:3,1:3),T1(1:3,4),prob.si.DH,prob.rob.rtb.tool);
% generate initial motion
ps.sGuess=nan(length(prob.time),ps.nS);
accrec=nan(length(prob.time),ps.nS/2);
for i=1:ps.nS/2
%     [pos,vel,acc] = motiongen(prob.init(i),prob.target(i),prob.dt,prob.tf);
    
    [pos,vel,acc] = motiongen(prob.init(i),mid(i),prob.dt,prob.tf/2-prob.dt);
    [pos1,vel1,acc1] = motiongen(mid(i),prob.target(i),prob.dt,prob.tf/2);
    pos=[pos(:);pos1(:)];vel=[vel(:);vel1(:)];acc=[acc(:);acc1(:)];

    ps.sGuess(:,i) = pos(:);
    ps.sGuess(:,6+i) = vel(:);
    accrec(:,i)=acc(:);
end
prob.u_init=prob.rob.rtb.rne(ps.sGuess(:,1:6),ps.sGuess(:,7:12),accrec);
[prob.lb,prob.ub,prob.bnds]=bounds(ps.npts,prob.init,prob.target,prob.rob,prob);
% solve optimal control
[opt.Xc,opt.Uc,opt.Tc,opt.Xopt,opt.Uopt,opt.Topt] = ...
    ps.timeOptimalOptimize(prob.lb,prob.ub,prob.time,prob.u_init);
end

