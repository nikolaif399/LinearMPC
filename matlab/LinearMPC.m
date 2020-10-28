classdef LinearMPC < handle
    %LinearMPC Setup and solve MPC problems with support for OSQP,
    %qpOASES and quadprog
    
    properties
        Ad % A matrix in discrete state space form
        Bd % B matrix in discrete state space form
        Q % Weight matrix on state deviation from states 0 -> N-1
        Qn % Weight matrix on state deviation at state N
        R % Weight matrix on control inputs
        StateBounds % Nx x 2 matrix of lower and upper bounds on state vars
        ControlBounds % Nu x 2 matrix of lower and upper bounds on control vars
        N % Control Intervals 
        Nx % Number of states
        Nu % Number of controls
        Nq % Toal number of decision variables
        
        H % Quadratic term in cost function
        f % Linear term in cost function
    end
    
    methods
        function obj = LinearMPC(Ad,Bd,Q,Qn,R,stateBounds,controlBounds,N)
            %LinearMPC Construct an instance of this class
            obj.Ad = Ad;
            obj.Bd = Bd;
            obj.Q = Q;
            obj.Qn = Qn;
            obj.R = R;
            obj.StateBounds = stateBounds;
            obj.ControlBounds = controlBounds;
            obj.Nx = size(Bd,1);
            obj.Nu = size(Bd,2);
            obj.H = nan;
            obj.f = nan;
            
            obj.updateHorizonLength(N)
        end
        
        function [] = updateHorizonLength(obj,N)
            obj.N = N;
            obj.Nq = (N+1)*(obj.Nx) + N*(obj.Nu);
        end
        
        function [] = setupCostFunction(obj,ref_traj)
            %setupCostFunction Constructs matrix H and vector f
            %   ref_traj Nx x N matrix with desired states
            Hq = kron(eye(obj.N),obj.Q);
            Hqn = obj.Qn;
            Hu = kron(eye(obj.N),obj.R);

            obj.H = blkdiag(Hq,Hqn,Hu);
            
            y = ref_traj(:);
            
            fx = y'*blkdiag(kron(eye(obj.N),obj.Q),obj.Qn);
            fu = zeros(obj.N*obj.Nu,1);
            obj.f = -[fx';fu];
        end
        
        function [Aeq,beq] = getEqualityConstraints(obj,initialState)
            % Aeq*x <= beq
            % This enforces initial state constraints as well as dynamics
            % constraints
            initialState = reshape(initialState,[obj.Nx 1]);
            
            A_padded_eye = padarray(kron(eye(obj.N), -eye(obj.Nx,obj.Nx)), [0 obj.Nx],0,'pre');
            A_padded_ad = padarray(kron(eye(obj.N),obj.Ad),[0,obj.Nx],0,'post');
            
            Adyn = [A_padded_eye + A_padded_ad, kron(eye(obj.N),obj.Bd)];
            Ainit = eye(obj.Nx,obj.Nx);
            Aeq = [Ainit zeros(obj.Nx,obj.Nq - obj.Nx);Adyn];
            
            beq = [initialState;zeros(obj.N*obj.Nx,1)];
        end
        
        function [A,lbA,ubA] = getDoubleSidedInequalityConstraints(obj)
            % lbA <= Ax <= ubA
            
            % Enforces state and control constraints
            A = [eye(obj.Nq,obj.Nq);eye(obj.Nq,obj.Nq)];
            [lbA,ubA] = obj.getSimpleBounds();
        end
    
        function [lb,ub] = getSimpleBounds(obj)
            % lb <= x <= ub
            
            % Enforce state and control constraints
            xmin = obj.StateBounds(:,1);
            xmax = obj.StateBounds(:,2);
            umin = obj.ControlBounds(:,1);
            umax = obj.ControlBounds(:,2);
            
            lb = [repmat(xmin,obj.N+1,1);repmat(umin,obj.N,1)];
            ub = [repmat(xmax,obj.N+1,1);repmat(umax,obj.N,1)];
            
        end
        
        function [H,f,A,b,Aeq,beq,lb,ub] = getQuadprogMatrices(obj, initialState, refTraj)
            obj.setupCostFunction(refTraj);
            H = obj.H;
            f = obj.f;
            A = [];
            b = [];
            [Aeq,beq] = obj.getEqualityConstraints(initialState);
            [lb,ub] = obj.getSimpleBounds();
        end
        
    end
end

