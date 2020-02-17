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
% [4] 6-axis robot, time optimal under dynamic constraints. Robot velocity,
% torque and torque rate are considered. Articulated body algorithm (ABA)
% is implemented to calculate robot forward dynamics, which is used as
% dynamic constraints of optimal control problem.
% [5] 6-axis robot, time optimal under dynamic constraints. Robot
% velocity, torque and torque rate are considered. Obstacle avoidance is
% considered. Robot links are constrained to be above ground.
% [6] 6-axis robot, time optimal under kinematic constraints and obstacle
% avoidance constraints. Robot position, velocity, acceleration, and jerk
% are bounded. Robot links are constrained to be above ground.

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
addpath(genpath('Examples/FromARTE'));
message = {"Available demos:",...
   sprintf('\t[0] 2D scara robot, time optimal under kinematic constraints'),...
   sprintf('\t[1] 2D scara robot, time optimal under obstacle and kinematic constraints'),...
   sprintf('\t[2] 2D scara robot, time optimal under obstacle and dynamic constraints'),...
   sprintf('\t[3] 2D wafer handling robot, time optimal under obstacle and kinematic constraints')...
   sprintf('\t[4] 6-axis robot, time optimal under dynamic constraints')...
   sprintf('\t[5] 6-axis robot, time optimal under dynamic constraints and obstacle avoidance')...
   sprintf('\t[6] 6-axis robot, time optimal under kinematic constraints and obstacle avoidance')...
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
        regul=input('Using regulation? [y/n] ','s');
        disp("  ");
        addpath('Examples/M20-dyn-simple');
        run('demo_M20dyn.m');
        rmpath('Examples/M20-dyn-simple');
    case 5
        disp("Your choice is: "+string(message{choice+2}(2:end)));
        disp("  ");
        regul=input('Using regulation? [y/n] ','s');
        disp("  ");
        showbndball=input('Show robot bounding ball? [y/n] ','s');
        disp("  ");
        addpath('Examples/M20-dyn-obstacle');
        run('demo_M20dyn_obs.m');
        rmpath('Examples/M20-dyn-obstacle');
    case 6
        disp("Your choice is: "+string(message{choice+2}(2:end)));
        disp("  ");
        showbndball=input('Show robot bounding ball? [y/n] ','s');
        disp("  ");
        addpath('Examples/M20-kin-obstacle');
        run('demo_M20kin_obs.m');
        rmpath('Examples/M20-kin-obstacle');
    otherwise
        disp("Invalid choice. Please try again.");
end