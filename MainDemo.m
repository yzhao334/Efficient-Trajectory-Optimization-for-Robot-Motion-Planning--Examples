%Demo code of: efficient trajectory optimization for robot motion planning
% Examples in this demo:
% [0] 2D scara robot, time optimal motion with kinematics constraints
% [1] 2D scara robot, obstacle avoidance with kinematics constraints
% [2] 2D scara robot, obstacle avoidance with dynamics constraints
% [3] 2D wafer handling robot, obscatle avoidance with kinematics
% constraints, description refer to "Trajectory planning for robot 
% manipulators considering kinematic constraints using probabilistic 
% roadmap approach." Xiaowen Yu etc., 2017, or "Intelligent Control and 
% Planning for Industrial Robots." Yu Zhao, 2018.
% [4] 3D robot demo ...

% optimal control based trajectory planning.
%   In this example, 3d 6-joint robot dynamics is considered, with
%   torque as control. Planning result is optimal motion and control for
%   time-optimally reaching target position under position, jerk, velocity,
%   acceleration, torque, and torque rate bound.
%   Self collision and workspace boundary is considered
% 
% After initialization, fast optimization can be performed when target
% changed.
clear all;clc;close all;
addpath(genpath('./Core'));
message = {"Available demos:",...
   sprintf('\t[0] 2D scara robot, time optimal under kinematic constraints'),...
   sprintf('\t[1] 2D scara robot, time optimal under obstacle and kinematic constraints'),...
   sprintf('\t[2] 2D scara robot, time optimal under obstacle and dynamic constraints'),...
   sprintf('\t[3] 2D wafer handling robot, time optimal under obstacle and kinematic constraints')...
   sprintf('\t[4] 6-axis robot, time optimal under dynamic constraints')...
   sprintf('Please choose one:  '),...
   };
for i = 1:length(message)
    disp(message{i});
end
choice = str2double(input(sprintf('\t'),'s'));
if isnan(choice)
    disp("Invalid choice. Please try again.");
    return;
end
switch choice
    case 0
        disp("Your choice is: "+string(message{choice+2}(2:end)));
        disp("  ");
        tmp=input('Press any key to start ');
        disp("  ");
        addpath('Examples/test2D-simple');
        run('demo_2Dsimple.m');
        rmpath('Examples/test2D-simple');
    case 1
        disp("Your choice is: "+string(message{choice+2}(2:end)));
        disp("  ");
        tmp=input('Press any key to start ');
        disp("  ");
        addpath('Examples/test2D-kin-obstacle');
        run('demo_2DKinObj.m');
        rmpath('Examples/test2D-kin-obstacle');
    case 2
        disp("Your choice is: "+string(message{choice+2}(2:end)));
        disp("  ");
        tmp=input('Press any key to start ');
        disp("  ");
        addpath('Examples/test2D-dyn-obstacle');
        run('demo_2DdynObj.m');
        rmpath('Examples/test2D-dyn-obstacle');
    case 3
        disp("Your choice is: "+string(message{choice+2}(2:end)));
        disp("  ");
        tmp=input('Press any key to start ');
        disp("  ");
        addpath('Examples/test-Wafer-3axis');
        run('demo_wafer.m');
        rmpath('Examples/test-Wafer-3axis');
    case 4
        disp("Your choice is: "+string(message{choice+2}(2:end)));
        disp("  ");
        tmp=input('Press any key to start ');
        disp("  ");
        addpath(genpath('Examples/FromARTE'));
        addpath('Examples/M20-dyn-simple');
        run('demo_M20dyn.m');
        rmpath('Examples/M20-dyn-simple');
        rmpath(genpath('Examples/FromARTE'));
    otherwise
        disp("Invalid choice. Please try again.");
end
rmpath(genpath('./Core'));