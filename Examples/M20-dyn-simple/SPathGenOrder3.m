function [Rs, v_Ct, a_Ct, t_end] = SPathGenOrder3(tLen, dt, pauseT)
% third order time optimal offline trjgen
% with manually chosen shape.

%  Rs : Generated vector of path primitive (s), the length along the path
%  v_Ct: Generated vector of the velocity along the path
%  a_Ct: Generated vector of the acceleration along the path
%  t_end: Total time duration along the path
%  tLen:   time length during motion
%  dt:     sample time
%  pauseT: pause time before motion

% Rs move from 0 to 1
t_end = tLen + pauseT;
time = 0:dt:t_end;
pos_range = [0,1];
time_range = [pauseT, time(end)];
Rs = nan(size(time_range));
v_Ct = nan(size(time_range));
a_Ct = nan(size(time_range));
for i = 1:length(time)
    [Rs(i),v_Ct(i),a_Ct(i),~,~,~,~] = trjgen(pos_range, time_range, time(i));
end
    

    
end
