function [ Pos,Vel,Acc,Jerk, velmax,accmax,jerkmax ] = trjgen( pos_range,time_range,time )
%TRJGEN simple trajectory generation program, [ Pos,Vel,Acc,Jerk, velmax,accmax,jerkmax ] = trjgen( pos_range,time_range,time )
% by Yu Zhao
pos_ini=pos_range(1);
pos_end=pos_range(2);
pos=pos_end-pos_ini;
t_ini=time_range(1);
t_end=time_range(2);
T=(t_end-t_ini)/3;
a=pos/T^3;
t=time-t_ini;

jerkmax=abs(a)*2;
accmax=abs(a)*T;
velmax=(3*T^2*abs(a))/4;
if (t>=0)&&(t<T)
    Jerk=a;
    Acc=a*t;
    Vel=1/2*a*t^2;
    Pos=(a*t^3)/6;    
elseif (t>=T)&&(t<2*T)
    Jerk=-2*a;
    Acc=-2*a*t+3*a*T;
    Vel=-a*t^2+3*a*T*t-3*a*T^2/2;
    Pos=(a*(3*T^3 - 9*T^2*t + 9*T*t^2 - 2*t^3))/6;
elseif (t>=2*T)&&(t<=3*T)
    Jerk=a;
    Acc=a*t-3*a*T;
    Vel=9*a*T^2/2-3*a*T*t+a*t^2/2;
    Pos=-(a*(21*T^3 - 27*T^2*t + 9*T*t^2 - t^3))/6;
elseif t>3*T
    Jerk=0;
    Acc=0;
    Vel=0;
    Pos=pos;
else
    %disp('time out of range');
    %Jerk=nan;
    %Acc=nan;
    %Vel=nan;
    %Pos=nan;
    Pos=0;
    Vel=0;
    Acc=0;
    Jerk=0;
    return;
end

Pos=Pos+pos_ini;


end

