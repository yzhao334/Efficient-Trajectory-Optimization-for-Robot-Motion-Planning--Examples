function rob = rob_params( )
% function for 2d 2-link robot parameter setup
% data fields:
%   kin: kinematic parameters
%       l1: length of link 1 (m unit)
%       l2: length of link 2 (m unit)
rob.kin.l1 = 1;
rob.kin.l2 = 0.75;
rob.kin.br = 0.1;
rob.kin.l1pts=[0.05,0.3,0.5,0.7,0.95];
rob.kin.l2pts=[0.05,0.3,0.5,0.7];
% obstacle parameter : [xc,yc,r]
rob.obs(1,:) = [-1,1,0.7];
rob.obs(2,:) = [2,0,0.5];
rob.obs(3,:) = [1.5,1.5,0.8];
rob.obs(4,:) = [0.5,-1,0.5];
% dynamic parameter
rob.dyn.r1 = rob.kin.l1/2;
rob.dyn.r2 = rob.kin.l2/2;
rob.dyn.rho = 9.8;
rob.dyn.m1 = rob.dyn.rho*rob.kin.l1;
rob.dyn.m2 = rob.dyn.rho*rob.kin.l2;
rob.dyn.I1 = 1/12*rob.dyn.m1*rob.kin.l1^2;
rob.dyn.I2 = 1/12*rob.dyn.m2*rob.kin.l2^2;
l1=rob.kin.l1;
r1=rob.dyn.r1;r2=rob.dyn.r2;
m1=rob.dyn.m1;m2=rob.dyn.m2;
I1=rob.dyn.I1;I2=rob.dyn.I2;
% dynamic equation see textbook by Li & Sastry
rob.dyn.alpha = I1+I2+m1*r1^2+m2*(l1^2+r2^2);
rob.dyn.beta = m2*l1*r2;
rob.dyn.delta = I2+m2*r2^2;


end
