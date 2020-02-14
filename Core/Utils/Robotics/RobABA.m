classdef RobABA < RobotObj
    %articulated body algorithm (ABA) and related for robot object
    %   main contents: ABA, RNE
    % author: Yu Zhao, yzhao334@berkeley.edu
    
    methods
        function obj = RobABA(modelname)
            obj = obj@RobotObj(modelname);
        end
        
        % inverse dynamics: recursive newton euler using spatial algebra
        function [ Torque ] = rne_s( obj, Pos, Vel, Acc )
            % transfer variables
            n=obj.r.n;
            Torque=Pos(:);
            gravity=[0;0;0;obj.r.gravity(:)];
            q=Pos(:)+obj.r.Theta0(:);% add zero, pos vector
            qd=Vel(:);% vel vector
            qdd=Acc(:);% acc vector
            Si = [0;0;1;0;0;0];
            % temp vectors
            Xup=cell(1,n);
            v=cell(1,n);
            a=cell(1,n);
            f=cell(1,n);
            % forward recursion 
            for j=1:n
                Xup{j} = obj.jcalc(j,q(j));
                vJ = Si*qd(j);
                if j==1
                    v{j} = Xup{j}*vJ;% caution: DH convention differ from spatial_v2
                    a{j} = Xup{j}*(gravity + Si*qdd(j));% caution: DH convention differ from spatial_v2
                else
                    v{j} = Xup{j}*(v{j-1} + vJ);% caution: DH convention differ from spatial_v2
                    a{j} = Xup{j}*(a{j-1} + Si*qdd(j) + RobABA.crm(v{j-1})*vJ);% caution: DH convention differ from spatial_v2
                end
                I=RobABA.mcI(obj.r.Ml(j),obj.r.Rl(:,j),obj.r.Jlmat(:,:,j));
                f{j} = I*a{j} + RobABA.crf(v{j})*I*v{j};
            end
            % backward recursion
            for j=n:-1:1
                Torque(j) = dot(Xup{j}*Si, f{j});% caution: DH convention differ from spatial_v2
                if j~=1
                    f{j-1} = f{j-1} + Xup{j}.'*f{j};
                end
            end
        end
        
        % inverse dynamics for gravity torque
        function [ Torque ] = gra_s( obj, Pos )
            % transfer variables
            n=obj.r.n;
            Torque=Pos(:);
            gravity=[0;0;0;obj.r.gravity(:)];
            q=Pos(:)+obj.r.Theta0(:);% add zero, pos vector
            Si = [0;0;1;0;0;0];
            % temp vectors
            Xup=cell(1,n);
            a=cell(1,n);
            f=cell(1,n);
            % forward recursion 
            for j=1:n
                Xup{j} = obj.jcalc(j,q(j));
                if j==1
                    a{j} = Xup{j}*(gravity);% caution: DH convention differ from spatial_v2
                else
                    a{j} = Xup{j}*(a{j-1});% caution: DH convention differ from spatial_v2
                end
                I=RobABA.mcI(obj.r.Ml(j),obj.r.Rl(:,j),obj.r.Jlmat(:,:,j));
                f{j} = I*a{j};
            end
            % backward recursion
            for j=n:-1:1
                Torque(j) = dot(Xup{j}*Si, f{j});% caution: DH convention differ from spatial_v2
                if j~=1
                    f{j-1} = f{j-1} + Xup{j}.'*f{j};
                end
            end
        end
        
        % forward dynamics: articulated body algorithm (ABA) using spatial algebra
        % considering motor inertia
        function [ Acc ] = aba_s( obj, Pos, Vel, Torque )
            % transfer variables
            n=obj.r.n;
            Acc=Pos(:);
            gravity=[0;0;0;obj.r.gravity(:)];
            q=Pos(:)+obj.r.Theta0(:);% add zero, pos vector
            qd=Vel(:);% vel vector
            Si = [0;0;1;0;0;0];
            % temp vectors
            Xup=cell(1,n);
            v=cell(1,n);
            c=cell(1,n);
            pA=cell(1,n);
            I=cell(1,n);
            U=cell(1,n);
            d=Pos(:);
            u=Torque(:);
            a=cell(1,n);
            % forward recursion 
            for j=1:n
                Xup{j} = obj.jcalc(j,q(j));
                vJ = Si*qd(j);
                if j==1
                    v{j} = Xup{j}*vJ;% caution: DH convention differ from spatial_v2
                    c{j} = [0;0;0;0;0;0];% caution: DH convention differ from spatial_v2
                else
                    v{j} = Xup{j}*(v{j-1} + vJ);% caution: DH convention differ from spatial_v2
                    c{j} = Xup{j}*(RobABA.crm(v{j-1}) * vJ);% caution: DH convention differ from spatial_v2
                end
                I{j}=RobABA.mcI(obj.r.Ml(j),obj.r.Rl(:,j),obj.r.Jlmat(:,:,j));
                pA{j} = RobABA.crf(v{j})*I{j}*v{j};
            end
            % backward recursion
            for j = n:-1:1
                Sj = Xup{j}*Si;
                U{j} = I{j} * Sj;
                d(j) = dot(Sj, U{j});
                u(j) = Torque(j) - dot(Sj,pA{j});
                if j~=1
                    Ia = I{j} - U{j}/d(j)*U{j}.';
                    pa = pA{j} + Ia*c{j} + U{j} * u(j)/d(j);
                    I{j-1} = I{j-1} + Xup{j}.' * Ia * Xup{j};
                    pA{j-1} = pA{j-1} + Xup{j}.' * pa;
                end
            end
            % forward recursion second round
            for j = 1:n
                if j==1
                    a{j} = Xup{j} * gravity + c{j};
                else
                    a{j} = Xup{j} * a{j-1} + c{j};
                end
                Acc(j) = (u(j) - dot(U{j},a{j}))/d(j);
                a{j} = a{j} + Xup{j}*Si*Acc(j);
            end
        end
        
        % get tcp acceleration (translational) by spatial algorithm
        % AccTcp: tcp acceleration in world frame
        % VelTcp: tcp velocity in world frame
        % AccTcpn: tcp acceleration (rotated to) in tool frame
        % VelTcpn: tcp velocity (rotated to) in tool frame
        function [AccTcp, VelTcp, AccTcpn, VelTcpn] = acc_s( obj, Pos, Vel, Acc )
            % transfer variables
            n=obj.r.n;
            q=Pos(:)+obj.r.Theta0(:);% add zero, pos vector
            qd=Vel(:);% vel vector
            qdd=Acc(:);% acc vector
            Si = [0;0;1;0;0;0];
            % temp vectors
            Xup=cell(1,n);
            v=cell(1,n);
            a=cell(1,n);
            % forward recursion 
            for j=1:n
                if j~=n
                    Xup{j} = obj.jcalc(j,q(j));
                else
                    Xup{j} = obj.toolcalc*obj.jcalc(j,q(j));% include tool frame
                end
                vJ = Si*qd(j);
                if j==1
                    v{j} = Xup{j}*vJ;% caution: DH convention differ from spatial_v2
                    a{j} = Xup{j}*(Si*qdd(j));% caution: DH convention differ from spatial_v2
                else
                    v{j} = Xup{j}*(v{j-1} + vJ);% caution: DH convention differ from spatial_v2
                    a{j} = Xup{j}*(a{j-1} + Si*qdd(j) + RobABA.crm(v{j-1})*vJ);% caution: DH convention differ from spatial_v2
                end
            end
            VelTcpn = v{n};
            AccTcp = a{n}(4:6) + cross(v{n}(1:3),v{n}(4:6));% conventional acceleration
            AccTcpn = AccTcp;
            temp = Xup{6}*Xup{5}*Xup{4}*Xup{3}*Xup{2}*Xup{1};% get rotation matrix;
            AccTcp = temp(1:3,1:3).'*AccTcp;
            VelTcp = [temp(1:3,1:3).'*VelTcpn(1:3);temp(1:3,1:3).'*VelTcpn(4:6)];
        end
        
        % vectorized forward dynamics
        function [ ret ] = FDynVec( obj, PosVelVec, TorqueVec )
            % PosVelVec: [PosVec;VelVec], see JTrqVec
            % torquevec see JTrqVec
            % VelAccVec=[velvec;accvec];
            % return vector of VelAccVec (i.e.VelAccVec(:)) as derivative of state
            N = size(PosVelVec, 2);
            ret = [];
            for k=1:N
                ret = [ret; ...
                        PosVelVec(obj.r.n+1:2*obj.r.n,k);...
                        obj.aba_s(PosVelVec(1:obj.r.n,k),...
                                 PosVelVec(obj.r.n+1:2*obj.r.n,k),...
                                 TorqueVec(:,k));...
                       ];
            end
        end
                
        % joint transformation for motion
        function XJ = jcalc( obj, j, q )
            c = cos(q);
            s = sin(q);
            ca = cos(obj.r.Alpha(j));
            sa = sin(obj.r.Alpha(j));
            E = [c, -s*ca, s*sa;...
                 s, c*ca,  -c*sa;...
                 0, sa,    ca].'; % E = R.'
            r = [obj.r.A(j)*c;obj.r.A(j)*s;obj.r.D(j)];
            XJ = [E,zeros(3);-E*RobABA.hat(r),E];
        end
        
        % tool frame transformation
        function XJ = toolcalc( obj )
            if isfield(obj.r,'tool')
                tool = obj.r.tool;
            else
                tool = eye(4);
            end
            E = (tool(1:3,1:3)).';
            r = tool(1:3,4);
            XJ = [E,zeros(3);-E*RobABA.hat(r),E];
        end

    end
    
    % wrap spatial_v2 functions
    methods(Static)
        % vector to matrix for cross product
        function ret = hat(w)
            ret=[0      -w(3)   w(2);...
                 w(3)   0       -w(1);...
                 -w(2)  w(1)    0];
        end
        
        % cross product of motion
        function ret = crm(v)
            ret = [  0    -v(3)  v(2)   0     0     0    ;...
                    v(3)  0    -v(1)   0     0     0    ;...
                    -v(2)  v(1)  0      0     0     0    ;...
                    0    -v(6)  v(5)   0    -v(3)  v(2) ;...
                    v(6)  0    -v(4)   v(3)  0    -v(1) ;...
                    -v(5)  v(4)  0     -v(2)  v(1)  0 ];
        end
        
        % cross product of motion
        function ret = crf(v)
            ret = -RobABA.crm(v).';
        end
        
        % spatial inertia matrix
        function ret = mcI( m, c, I )
            C = RobABA.hat(c);
            ret = [ I + m*(C*C.'), m*C; m*C.', m*eye(3) ];
        end
    end
    
end

