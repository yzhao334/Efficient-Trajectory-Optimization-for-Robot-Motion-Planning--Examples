classdef RobotObj < handle
    %wrapper of robot object for parameter setting
    % author: Yu Zhao, yzhao334@berkeley.edu
    
    % public properties
    properties
        ModelName; % model name of of robot
        r;  % struct contains robot parameters
        bnd_ball;
    end
    
    methods
        % constructor
        function obj = RobotObj(model)
            switch model
                case 'M20iA' % M20iA robot model from ARTE
                    obj = obj.buildM20iA;
                otherwise
                    error('Robot model not supported yet.');
            end
            obj.ModelName=model;
        end
        
        % build M20iA model
        function obj = buildM20iA(obj)
            % useful inline functions
%             % skew matrix for cross product
%             hat=@(w)[0      -w(3)   w(2);...
%              w(3)   0       -w(1);...
%              -w(2)  w(1)    0];
            % from [Ixx Iyy Izz Ixy Iyz Ixz] to I matrix
            v2M=@(x)[x(1)   x(4)    x(6);...
                     x(4)   x(2)    x(5);...
                     x(6)   x(5)    x(3)];
            
            obj.r.name = 'M20iA';
            obj.r.manufacturer = 'FANUC Corporation';
            obj.r.n = 6;
            % important: should set path before run
            obj.r.path = which('inversekinematic_fanuc_m_20ia.m');
            obj.r.path=obj.r.path(1:end-31);
            [obj.r.cad.base{1}.f,obj.r.cad.base{1}.v]=...
                stlread([obj.r.path,'link0_exp.stl']);
            obj.r.cad.base{1}.color = [0.4,0.4,0.4];
            for j=1:6
                [obj.r.cad.link{j}.f,obj.r.cad.link{j}.v]=...
                    stlread([obj.r.path,'link',num2str(j),'_exp.stl']);
            end
            for j=1:5
                obj.r.cad.link{j}.color = 'y';
            end
            obj.r.cad.link{6}.color = 'k';

            % DH parameters
            obj.r.Theta0  = [0 -pi/2 0 0 0 0]; % jnt1 use different value than ARTE                                    % Joint variable offset (rad)
            obj.r.D       = [0.525 0 0 0.835 0 0.100];                                  % Joint extension (m)
            obj.r.A       = [0.150 0.790 0.250 0 0 0];                                  % Joint offset (m)
            obj.r.Alpha   = [-pi/2 0 -pi/2 pi/2 -pi/2 0];                               % Joint twist (rad)
            obj.r.Qlim    = [-185 185; -100 110; -229 229; ...
                        -200 200; -180 180; -450 450]'*pi/180; % modified jnt angle limit of jnt2                     % Joint angle limit (rad)
            obj.r.QDlim   = [-195 195; -175 175; -180 180; ...
                        -360 360; -360 360; -550 550]'*pi/180;
                    
            obj.r.T_B2W = eye(4);

            % Reducer parameters
            obj.r.G       = [10 6 42 11 2 2];                                       % Gear ratio
            
            % Link parameters, ignoring any friction for now
            obj.r.Ml = [84 38 44 28 1.8 0.2];         % Link mass (kg)
            obj.r.Rl = [-0.125     0.05    0.05;
                    -0.425	  0	    -0.15;
                    -0.125	-0.025  0.075;
                    0        -0.26   -0.03;
                    0         0      0.04;
                    0         0      -0.02].';      % Link center of gravity relative to each joint's frame (m)
            
            linkInertia = [2.1700      3.5000	    3.0100      0.5250  	-0.2100	 0.5250;
                        1.0133      10.6083 	9.8483	    0	         0  	-2.4225;
                        0.7333  	1.6133	    1.3933	   -0.1375	     0.0825	 0.4125;                   
                        3.0417	    0.1148	    3.0165	    0	        -0.2184	 0;
                        0.0055	    0.0055	    0.0032	    0          	0	     0;
                        1.5170e-04	1.5170e-04	9.0000e-05	0	        0	     0];
            for j=1:6
                obj.r.Jlmat(:,:,j) = v2M(linkInertia(j,:)); % in link frame
                obj.r.Jlmat(:,:,j)=Inertia_parallel_axis_toCog(...
                    obj.r.Jlmat(:,:,j),obj.r.Ml(j),-obj.r.Rl(:,j)); % in cog frame
            end
            
            % Gravity
            obj.r.gravity     = [0; 0; 9.81];                                        % Gravity vector in world coordinates (m/sec^2)
            
            % some common poses
            obj.r.qz  = [0 0 0 0 0 0];         % zero angles, L shaped pose
            obj.r.qh  = [0 0 0 0 0 0];         % home pose
            
            % setup bounding balls for collision detection
            obj.bnd_ball.c{1}=[0 0 0.1];
            obj.bnd_ball.r{1}=0.26;
            
            obj.bnd_ball.c{2}=[-0.1,0.08,0.034];
            obj.bnd_ball.r{2}=0.29;
            
            obj.bnd_ball.c{3}(1,:)=[-0.06,0,-0.17];
            obj.bnd_ball.r{3}(1)=0.17;
            obj.bnd_ball.c{3}(2,:)=[-0.33,0,-0.19];
            obj.bnd_ball.r{3}(2)=0.16;
            obj.bnd_ball.c{3}(3,:)=[-0.59,0,-0.16];
            obj.bnd_ball.r{3}(3)=0.16;
            obj.bnd_ball.c{3}(4,:)=[-0.8,0,-0.16];
            obj.bnd_ball.r{3}(4)=0.16;
            
            obj.bnd_ball.c{4}(1,:)=[0,0,0.11];
            obj.bnd_ball.r{4}(1)=0.15;
            obj.bnd_ball.c{4}(2,:)=[-0.23,-0.01,0.07];
            obj.bnd_ball.r{4}(2)=0.22;
            
            obj.bnd_ball.c{5}(1,:)=[0,-0.47,0];
            obj.bnd_ball.r{5}(1)=0.15;
            obj.bnd_ball.c{5}(2,:)=[0,-0.26,-0.05];
            obj.bnd_ball.r{5}(2)=0.15;
            obj.bnd_ball.c{5}(3,:)=[0,-0.025,-0.04];
            obj.bnd_ball.r{5}(3)=0.16;
            
            obj.bnd_ball.c{6}=[];
            obj.bnd_ball.r{6}=[];
            obj.bnd_ball.c{7}=[];
            obj.bnd_ball.r{7}=[];
        end 
        
        % analytical inverse kinematics of M20iA
        % using DH parameter from ARTE, except Theta0
        % current implementation cannot handle exceptions
        % input: R, P: tool frame orientation and position
        % output: q = joint position
        function [ theta ] = kinv_M20iA( obj,R,P )
        % initialize as row vector
        theta=nan(1,6);
        % DH parameters
        alpha=obj.r.Alpha;
        A=obj.r.A;
        D=obj.r.D;
        offset=obj.r.Theta0;
        % Tool frame
        if isfield(obj.r,'tool')
            R_tool=tool(1:3,1:3);
            T_tool=tool(1:3,4);
        else
            R_tool=eye(3);
            T_tool=[0;0;0];
        end
        %
        P = P(:);
        S6=[R,P;zeros(1,3),1]/[R_tool,T_tool;zeros(1,3),1];
        p=S6(1:3,4);
        R6=S6(1:3,1:3);
        w6=[0;0;D(6)];
        c=p-R6*w6;
        theta(1)=atan2(c(2),c(1));
        c0=c;
        c=[c(1)-A(1)*cos(theta(1));...
           c(2)-A(1)*sin(theta(1));...
           c(3)-D(1)];
        Lc=norm(c);
        L2=sqrt(A(3)^2+D(4)^2);
        Lc0=sqrt((A(2)+A(3))^2+D(4)^2);
        theta(3)=acos((L2^2+A(2)^2-Lc^2)/(2*L2*A(2)))-acos((L2^2+A(2)^2-Lc0^2)/(2*L2*A(2)));
        theta(3)=-theta(3);
        if ~isreal(theta(3))
            theta(3)=nan;
            return;
        end
        dir=[cos(theta(1)),sin(theta(1))];
        temp=dot(dir,[c0(1),c0(2)]);
        if temp>A(1)
            theta(2)=acos((A(2)^2+Lc^2-L2^2)/(2*A(2)*Lc))+atan(c(3)/sqrt(c(1)^2+c(2)^2));
        elseif temp<A(1)
            theta(2)=acos((A(2)^2+Lc^2-L2^2)/(2*A(2)*Lc))-atan(c(3)/sqrt(c(1)^2+c(2)^2))+pi;
        else
            theta(2)=acos((A(2)^2+Lc^2-L2^2)/(2*A(2)*Lc))+pi/2;
        end
        if ~isreal(theta(2))
            theta(2)=nan;
            return;
        end
        theta(2)=-theta(2)-obj.r.Theta0(2);
        q=theta;
        for i=1:3
            q(i)=q(i)+offset(i);
        end
        TCP_T=eye(4);
        for i=1:3    
            TCP_T=TCP_T*...
                [cos(q(i)) -sin(q(i))*cos(alpha(i))  sin(q(i))*sin(alpha(i)) A(i)*cos(q(i));...
                 sin(q(i))  cos(q(i))*cos(alpha(i)) -cos(q(i))*sin(alpha(i)) A(i)*sin(q(i));...
                  0            sin(alpha(i))                cos(alpha(i))            D(i);...
                  0                0                       0               1];
        end
        T456=TCP_T\S6;
        theta(5)=acos(T456(3,3));
        if sin(theta(5))~=0
            theta(4)=atan2(-T456(2,3)/sin(theta(5)),-T456(1,3)/sin(theta(5)));
            theta(6)=atan2(-T456(3,2)/sin(theta(5)),T456(3,1)/sin(theta(5)));
        elseif cos(theta(5))==1 % singuality
            theta(4)=nan;
            theta(5)=nan;
        end

        end

        
        % hand crafted newton euler, for casadi purpose
        function [ Torque ] = NewtonEuler( obj, Pos, Vel, Acc )
            % transfer variables
            n=obj.r.n;
            Torque=[];
            gravity=obj.r.gravity(:);
            q=Pos(:)+obj.r.Theta0(:);% add zero, pos vector
            qd=Vel(:);% vel vector
            qdd=Acc(:);% acc vector
            % define temp variables
            ROT=cell(1,n);
            PSTAR=cell(1,n);
            OMEGA=cell(1,n);
            OMEGADOT=cell(1,n);
            ACC=cell(1,n);
            ACC_COG=cell(1,n);
            F=cell(1,n); %force
            Nt=cell(1,n); %torque
            % forward recursion 
            for j=1:n
                % rotation & translation for each link
                ROT{j}=[cos(q(j)), -sin(q(j))*cos(obj.r.Alpha(j)), sin(q(j))*sin(obj.r.Alpha(j));...
                     sin(q(j)), cos(q(j))*cos(obj.r.Alpha(j)),  -cos(q(j))*sin(obj.r.Alpha(j));...
                     0,         sin(obj.r.Alpha(j)),            cos(obj.r.Alpha(j))];
                PSTAR{j}=[obj.r.A(j),obj.r.D(j)*sin(obj.r.Alpha(j)),obj.r.D(j)*cos(obj.r.Alpha(j))].';
                % omega & omegadot
                if j==1
                    t1=[0,0,qd(j)].';
                    t3=[0,0,qdd(j)].';
                else
                    t1=OMEGA{j-1}+[0,0,qd(j)].';
                    t3=OMEGADOT{j-1}+[0,0,qdd(j)].'+...
                        cross(OMEGA{j-1},[0,0,qd(j)].');
                end
                OMEGA{j}=ROT{j}.'*t1;
                OMEGADOT{j}=ROT{j}.'*t3;
                % acc
                ACC{j}=cross(OMEGADOT{j},PSTAR{j})+...
                    cross(OMEGA{j},cross(OMEGA{j},PSTAR{j}));
                if j==1
                    t1=ROT{j}.'*gravity;
                else
                    t1=ROT{j}.'*ACC{j-1};
                end
                ACC{j}=ACC{j}+t1;
                % abar
                ACC_COG{j}=cross(OMEGADOT{j},obj.r.Rl(:,j))+...
                    cross(OMEGA{j},cross(OMEGA{j},obj.r.Rl(:,j)))+...
                    ACC{j};
            end
            % backward recursion 
            for j=n:-1:1
                % F
                t4=ACC_COG{j}*obj.r.Ml(j);
                if j~=n
                    F{j}=t4+ROT{j+1}*F{j+1};
                else
                    F{j}=t4;
                end
                % Nt
                t1=cross(PSTAR{j}+obj.r.Rl(:,j),t4);
                if j~=n
                    t1=ROT{j+1}*(...
                    cross(ROT{j+1}.'*PSTAR{j},F{j+1})+...
                        Nt{j+1}...
                    )+t1;
                end
                Nt{j}=t1+obj.r.Jlmat(:,:,j)*OMEGADOT{j}+...
                    cross(OMEGA{j},obj.r.Jlmat(:,:,j)*OMEGA{j});
            end
            % compute torque total for each axis
            for j=1:n
                t=dot(Nt{j},ROT{j}.'*[0,0,1].');
                % (currently ignore friction)
                Torque=[Torque;t];
            end 
        end
        
    end
    
end

