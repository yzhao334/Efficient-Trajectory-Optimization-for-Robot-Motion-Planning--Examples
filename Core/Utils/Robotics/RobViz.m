classdef RobViz < RobABA
    %ROBVIZ visualization class
    % author: Yu Zhao, yzhao334@berkeley.edu
    
    properties
        parts;
        h_rob;
        workspace; % box workspace as xlimit,ylimit,zlimit
        light;
        jnt_pos;
    end
    
    methods
        function obj = RobViz(name)
            obj = obj@RobABA(name);
            obj.loadCAD;
            obj.jnt_pos=obj.r.qh(:);% initial position
            
            obj.workspace.xlimit=[-0.5,1.5];
            obj.workspace.ylimit=[-0.5,0.5];
            obj.workspace.zlimit=[0,2];
        end
        
        function delete(obj)
            for i=1:length(obj.h_rob)
                if isgraphics(obj.h_rob(i))
                    delete(obj.h_rob(i));
                end
            end
            for j=1:length(obj.light)
                if isgraphics(obj.light(j))
                    delete(obj.light(j));
                end
            end
        end
        
        function loadCAD(obj)
            cad=obj.r.cad;
            obj.parts(1).data.faces=cad.base{1}.f;
            obj.parts(1).data.vertices=cad.base{1}.v;
            obj.parts(1).id=1;
            obj.parts(1).color=cad.base{1}.color;
            for i=1:length(cad.link)
                obj.parts(i+1).data.faces=cad.link{i}.f;
                obj.parts(i+1).data.vertices=cad.link{i}.v;
                obj.parts(i+1).id=i+1;
                obj.parts(i+1).color=cad.link{i}.color;
            end
            if isfield(cad,'payload')
                obj.parts(end+1).data.faces=cad.payload{1}.f;
                obj.parts(end).data.vertices=cad.payload{1}.v;
                obj.parts(end).id=length(obj.parts)-1;
                obj.parts(end).color=cad.payload{1}.color;
            end
        end
                
        % forward kinematics
        %
        % Outputs:  jntT         transform of each joint
        function [ jntT ] = kfwd_rob_full(obj)
            % DH parameters
            alpha=obj.r.Alpha;
            A=obj.r.A;
            D=obj.r.D;
            offset=obj.r.Theta0;
            %
            jntT = zeros(4,4,numel(obj.parts));
            jntT(:,:,1)=obj.r.T_B2W;
            q = obj.jnt_pos(:) + offset(:);
            TCP_T=jntT(:,:,1);
            for i=1:6
                TCP_T=TCP_T*...
                    [cos(q(i)) -sin(q(i))*cos(alpha(i))  sin(q(i))*sin(alpha(i)) A(i)*cos(q(i));...
                     sin(q(i))  cos(q(i))*cos(alpha(i)) -cos(q(i))*sin(alpha(i)) A(i)*sin(q(i));...
                      0            sin(alpha(i))                cos(alpha(i))            D(i);...
                      0                0                       0               1];
                jntT(:,:,i+1) = TCP_T;
            end
            if isfield(obj.r,'tool')
                TCP_T=TCP_T*... % up to now, forward kinematics, S0 to S6
                    obj.r.tool; % S6 to tool
                jntT(:,:,end) = TCP_T;
            end

        end
        
        function draw(obj,q)
            obj.jnt_pos=q(:);
            [jntT]=obj.kfwd_rob_full();
            for j=1:length(obj.parts)
                set(obj.h_rob(j),'vertices',bsxfun(@plus,...
                    obj.parts(j).data.vertices*...
                    jntT(1:3,1:3,obj.parts(j).id).',...
                    jntT(1:3,4,obj.parts(j).id).'));
            end
            drawnow;
        end
        
        function InitPlot(obj,viewang)
            clf;
            hold on;
            [jntT]=obj.kfwd_rob_full();
            partsnum=numel(obj.parts);
            obj.h_rob=nan(partsnum,1);
            for j=1:partsnum
                obj.h_rob(j)=patch('faces',obj.parts(j).data.faces,...
                    'vertices',bsxfun(@plus,obj.parts(j).data.vertices*...
                    jntT(1:3,1:3,obj.parts(j).id).',...
                    jntT(1:3,4,obj.parts(j).id).'));
                set(obj.h_rob(j),'facecolor',obj.parts(j).color);
                set(obj.h_rob(j),'edgecolor','none');
                set(obj.h_rob(j),'Clipping','off');
            end
            % set lighting, view, etc
            axis equal;
            view(viewang);
            xlim(obj.workspace.xlimit);
            ylim(obj.workspace.ylimit);
            zlim(obj.workspace.zlimit);
            axis vis3d;
            box on;grid on;
            obj.light=camlight('right');
        end        
        
    end
    
    methods(Static)
        % plot coordinate frame
        function h = frameplot( R, P, length )
            % plot frame
            R=R*length;
            hold on;
            % xaxis
            h(1)=quiver3(P(1),P(2),P(3),R(1,1),R(2,1),R(3,1),'r');
            % yaxis
            h(2)=quiver3(P(1),P(2),P(3),R(1,2),R(2,2),R(3,2),'g');
            % zaxis 
            h(3)=quiver3(P(1),P(2),P(3),R(1,3),R(2,3),R(3,3),'b');
        end
        
        % set coordinate frame
        function h = setframeplot( R, P, length, h )
            % plot frame
            R=R*length;
            for i=1:3
                set(h(i),'XData',P(1),...
                         'YData',P(2),...
                         'ZData',P(3),...
                         'UData',R(1,i),...
                         'VData',R(2,i),...
                         'WData',R(3,i));
            end

        end

    end
    
end

