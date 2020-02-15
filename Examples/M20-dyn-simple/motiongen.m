function [pos,vel,acc] = motiongen(pnt0,pnt1,dt,tLen)
    % 'toy' trajectory generator for smooth trajectory
    % author: Yu Zhao, yzhao334@berkeley.edu
    time0=0:dt:tLen;
    [Rs, v_Ct, a_Ct, ~] = SPathGenOrder3(time0(end), dt, 0);% generate normalized trajectory
    m = length([0:dt:tLen])-length(time0);
    Rs = [Rs(:);ones(m,1)];
    v_Ct = [v_Ct(:);zeros(m,1)];
    a_Ct = [a_Ct(:);zeros(m,1)];
    scale = (pnt1-pnt0);% scale of actual trajectory
    pos = Rs*scale+pnt0;% get actual pos
    vel = v_Ct*scale;% get actual vel
    acc = a_Ct*scale;% get actual acc
end