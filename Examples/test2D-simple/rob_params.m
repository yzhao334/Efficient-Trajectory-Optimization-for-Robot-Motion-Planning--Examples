function rob = rob_params( )
% function for 2d 2-link robot parameter setup
% data fields:
%   kin: kinematic parameters
%       l1: length of link 1 (m unit)
%       l2: length of link 2 (m unit)
rob.kin.l1 = 1;
rob.kin.l2 = 0.75;
rob.kin.br = 0.2/5;
rob.kin.l1pts=[0.1,0.5,0.9];
rob.kin.l2pts=[0.2,0.6];


end
