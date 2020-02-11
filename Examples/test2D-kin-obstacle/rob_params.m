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


end
