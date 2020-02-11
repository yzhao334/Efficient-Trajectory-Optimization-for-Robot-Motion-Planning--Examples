classdef ChebTest
    % functions for chebyshev collocation
    % dependency: chebfun, casadi
    % author: Yu Zhao, yzhao334@berkeley.edu
    
    properties
    end
    
    methods
    end
    
    methods(Static)
        function D = getDifferentiationMatrix(x,v,d)
            % D = getDifferentiationMatrix(x,v,d)
            %
            %
            % INPUTS:
            %   x = [n,1] = vector of roots of the orthogonal polynomial of interest
            %   v = [n,1] = vector of barycentric weights corresponding to each root
            %   d = [1,2] = domain of the polynomial (optional)
            %
            % OUTPUTS:
            %   D = [n,n] = differentiation matrix such that dy/dx = D*y @ points in x
            %
            % NOTES:
            %   Reference:
            %       1) ChebFun  (http://www.chebfun.org/)
            %       2) "Barycentric Lagrange Interpolation"   SIAM Review 2004
            %           Jean-Paul Berrut and Lloyd N. Trefethen
            %
            %   Inputs: x and v are typically produced by a call to any of:
            %       chebpts, trigpts, legpts, jacpts, lagpts, hermpts, lobpts, radaupts
            %

            if nargin == 2
                d = [-1,1];
            end

            n = length(x);
            D = zeros(n,n);
            for i=1:n
                D(i,:) = (v/v(i))./(x(i)-x);
                D(i,i) = 0;
                D(i,i) = -sum(D(i,:));
            end

            D = 2*D/(d(2)-d(1));
        end
        
        function y = barycentricInterpolate(x,yk,xk,vk)
            % y = barycentricInterpolate(x,yk,xk,vk)
            %
            % Interpolates an orthogonal polynomial using barycentric interpolation
            %
            % INPUTS:
            %   x = [nTime, 1] = vector of points to evaluate polynomial at
            %   yk = [nGrid, nOut] = value of the function to be interpolated at each
            %       grid point
            %   xk = [nGrid, 1] = roots of orthogonal polynomial
            %   vk = [nGrid, 1] = barycentric interpolation weights
            %
            % OUTPUTS:
            %   y = [nTime, nOut] = value of the function at the desired points
            %
            % NOTES:
            %   xk and yk should be produced by chebfun (help chebpts)
            %

            nOut = size(yk,2);
            nTime = length(x);
            y = zeros(nTime, nOut);

            for i=1:nOut
                y(:,i) = bary(x,yk(:,i),xk,vk);
            end
        end
        
        
        function [x,w,D] = orthScale(orth,d)
            % [x,w,D] = orthScale(orth,d)
            %
            % This function scales the chebyshev points to an arbitrary interval
            %
            % INPUTS:
            %   xx = chebyshev points on the domain [-1,1]
            %   ww = chebysehv weights on the domain [-1,1]
            %   d = [low, upp] = new domain
            %
            % OUTPUTS:
            %   x = chebyshev points on the new domain d
            %   w = chebyshev weights on the new domain d
            %

            shift = 0.5*(d(1) + d(2));
            scale = 0.5*(d(2) - d(1));

            x = scale*orth.xx + shift;

            if nargout > 1
                w = orth.ww*scale;
            end

            if nargout > 2
                D = orth.D/scale;
            end

        end
           
        
        function [t,x,u,w] = unPackDecVar(z,pack,orth)
            %
            % This function unpacks the decision variables for
            % trajectory optimization into the time (t),
            % state (x), and control (u) matricies
            %
            % INPUTS:
            %   z = column vector of 2 + nTime*(nState+nControl) decision variables
            %   pack = details about how to convert z back into t,x, and u
            %       .nTime
            %       .nState
            %       .nControl
            %
            % OUTPUTS:
            %   t = [1, nTime] = time vector (grid points)
            %   x = [nState, nTime] = state vector at each grid point
            %   u = [nControl, nTime] = control vector at each grid point
            %   w = [1, nTime] = weights for clenshaw-curtis quadrature
            %

            nTime = pack.nTime;
            nState = pack.nState;
            nControl = pack.nControl;
            nx = nState*nTime;
            nu = nControl*nTime;

            [t, w] = ChebTest.orthScale(orth,[z(1),z(2)]);
            t = t';
            x = reshape(z((2+1):(2+nx)),nState,nTime);
            u = reshape(z((2+nx+1):(2+nx+nu)),nControl,nTime);
        end
        
        function [z,pack] = packDecVar(t,x,u)
            %
            % This function collapses the time (t), state (x)
            % and control (u) matricies into a single vector
            %
            % INPUTS:
            %   t = [1, nTime] = time vector (grid points)
            %   x = [nState, nTime] = state vector at each grid point
            %   u = [nControl, nTime] = control vector at each grid point
            %
            % OUTPUTS:
            %   z = column vector of 2 + nTime*(nState+nControl) decision variables
            %   pack = details about how to convert z back into t,x, and u
            %       .nTime
            %       .nState
            %       .nControl
            %

            nTime = length(t);
            nState = size(x,1);
            nControl = size(u,1);

            tSpan = [t(1); t(end)];
            xCol = reshape(x, nState*nTime, 1);
            uCol = reshape(u, nControl*nTime, 1);

            z = [tSpan;xCol;uCol];

            pack.nTime = nTime;
            pack.nState = nState;
            pack.nControl = nControl;

        end
        
        % check how many points could approximate function/signal, 1-D
        function [ ucheb,udcheb ] = chebinterPosVel( u,time,npts )
            %chebshev interpolation to get pos and vel for u
            %   u:      original pos signal, 1-D
            %   time:   time of u
            %   npts:   # of points for interpolation
            Uc=u(:);
            time=time(:);
            [orth.xx,orth.ww,orth.vv]=chebpts(npts);% cheb pnts and weights
            orth.D=ChebTest.getDifferentiationMatrix(orth.xx,orth.vv);
            tc=chebpts(npts,[time(1),time(end)]);
            Uc=interp1(time,Uc,tc);
            scale=(time(end)-time(1))/2;
            Ucd=orth.D/scale*Uc(:);

            ucheb=ChebTest.barycentricInterpolate(time(:),Uc(:),tc(:),orth.vv(:));
            udcheb=ChebTest.barycentricInterpolate(time(:),Ucd(:),tc(:),orth.vv(:));


        end
        
        
        function Xsim = test(nColPts,time,posvelm,rob,initCond)
            [orth.xx, orth.ww, orth.vv] = chebpts(nColPts);
            orth.D = ChebTest.getDifferentiationMatrix(orth.xx,orth.vv);
            guess.tSpan = time([1,end]);
            guess.time = chebpts(nColPts,guess.tSpan)';
            guess.state = interp1(time(:), posvelm, guess.time')';
            dynfcn=@(x)rob.linkDyn210f(x,guess.time,time,posvelm);
            
            x0=guess.state(:);
            x0=x0(13:end);
            scale=(guess.tSpan(2)-guess.tSpan(1))/2;
            eqn=@(x)ChebTest.testEqn(x,orth.D,dynfcn,12,initCond,scale);
            opt=optimoptions('fsolve','Display','iter','Algorithm','levenberg-marquardt');
            xsoln=fsolve(eqn,x0,opt);
            xsoln=[initCond(:);xsoln];
            pack.nTime = nColPts;
            pack.nState = 12;
            pack.nControl = 0;
            [tSoln,xSoln,uSoln] = ChebTest.unPackDecVar([guess.tSpan(:);xsoln],pack,orth);
            %%%% Store the results:
            soln.grid.time = tSoln;
            soln.grid.state = xSoln;
            soln.grid.control = uSoln;
            
            %%%% Rescale the points:
            dSoln = tSoln([1,end]);   %Domain of the final solution
            xxSoln = ChebTest.orthScale(orth,dSoln);
            soln.interp.state = @(t)( ChebTest.barycentricInterpolate(t', xSoln',xxSoln,orth.vv)' );
            
            Xsim = soln.interp.state(time(:).');

        end
        
        
        function ret = testEqn(x,D,dynfcn,nS,x0,scale)
            x=reshape(x,nS,[]);
            x=[x0(:),x];
            dxFun = (D/scale*(x'))';   %Differentiate trajectory
            dxDyn = dynfcn(x);
            % Add a constraint that both versions of the derivative must match:
            defects = dxFun - dxDyn;
            ret = defects(:);
            
        end
        
        
    end
    
end

