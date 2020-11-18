classdef LinearMPC < handle
    %LinearMPC Setup and solve MPC problems with support for OSQP,
    %qpOASES and quadprog
    
    properties
        Solver % Solver to use 'quadprog' or 'qpoases'
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
        
        qpParams % Specific params for each solver
       
    end
    
    methods
        function obj = LinearMPC(Ad,Bd,Q,Qn,R,stateBounds,controlBounds,N,varargin)
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
            
            obj.updateHorizonLength(N)
            
            p = inputParser;
            addParameter(p,'Solver','quadprog',@(x) obj.solverNameValid(x));
            parse(p,varargin{:});
            obj.Solver = p.Results.Solver;
            fprintf('Using solver %s\n', obj.Solver);
            
            obj.setupSolver();
        end
        
        function [] = changeSolver(obj,solver)
            if obj.solverNameValid(solver)
                obj.Solver = solver;
                obj.setupSolver();
            end
             
        end
        
        function [] = setupSolver(obj)
            if strcmp(obj.Solver,'quadprog')
                
            elseif strcmp(obj.Solver,'qpoases')
                obj.qpParams.options = qpOASES_options('mpc', ...
                'printLevel', 0, 'maxIter', 1e8, 'maxCpuTime', 60, ...
                'initialStatusBounds', 0);
            end
        end
        
        function valid = solverNameValid(obj,s)
            if strcmp(s,'quadprog') || strcmp(s,'qpoases')
                valid = true;
            else
                warning("Unknown solver type " + s)
                valid = false;
            end
        end
        
        function [] = updateHorizonLength(obj,N)
            if N < 1
                error("N must be greater than zero")
            end
            obj.N = N;
            obj.Nq = (N+1)*(obj.Nx) + N*(obj.Nu);
        end
        
        function [H,f] = getCostFunction(obj,ref_traj)
            %setupCostFunction Constructs matrix H and vector f
            %   ref_traj Nx x N matrix with desired states
            Hq = kron(eye(obj.N),obj.Q);
            Hqn = obj.Qn;
            Hu = kron(eye(obj.N),obj.R);

            H = blkdiag(Hq,Hqn,Hu);
            
            y = ref_traj(:);
            
            fx = y'*blkdiag(kron(eye(obj.N),obj.Q),obj.Qn);
            fu = zeros(obj.N*obj.Nu,1);
            f = -[fx';fu];
        end
        
        function [Aeq,beq] = getDynamicsConstraint(obj)
            % Aeq*x = beq
            
            % Enforces dynamics constraints
            A_padded_eye = padarray(kron(eye(obj.N), -eye(obj.Nx,obj.Nx)), [0 obj.Nx],0,'pre');
            A_padded_ad = padarray(kron(eye(obj.N),obj.Ad),[0,obj.Nx],0,'post');
            
            Aeq = [A_padded_eye + A_padded_ad, kron(eye(obj.N),obj.Bd)];
            beq = zeros(obj.N*obj.Nx,1);
            
        end
        
        function [lb,ub] = getStateControlBounds(obj, initialState)
            % lb <= x <= ub
            
            % Enforce state bounds
            xmin = obj.StateBounds(:,1);
            xmax = obj.StateBounds(:,2);
            
            % Enforce control bounds
            umin = obj.ControlBounds(:,1);
            umax = obj.ControlBounds(:,2);
            
            lb = [initialState;repmat(xmin,obj.N,1);repmat(umin,obj.N,1)];
            ub = [initialState;repmat(xmax,obj.N,1);repmat(umax,obj.N,1)];
            
        end
       
        function [xout,fval] = solve(obj,initialState, refTraj)
            [H,f] = obj.getCostFunction(refTraj);
            
            if strcmp(obj.Solver,'quadprog')
                [Aeq,beq] = obj.getDynamicsConstraint();
                [lb,ub] = obj.getStateControlBounds(initialState);
                
                A = [];
                b = [];
                options.Algorithm = 'interior-point-convex';
                options.Display = 'off';
                iters = 100;
                tic
                for i = 1:iters
                    [xout,fval] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
                end
                no_hotstart = toc/iters
                
            elseif strcmp(obj.Solver,'qpoases')
                [A_dyn,b_dyn] = obj.getDynamicsConstraint();
                [lb,ub] = obj.getStateControlBounds(initialState);

                % Unlifted form
                al = b_dyn;
                A = A_dyn;
                au = b_dyn;

                xl = lb;
                xu = ub;
                tic
                [QP,xout,fval,exitflag,iter,lambda] = qpOASES_sequence('i',H,f,A,xl,xu,al,au,obj.qpParams.options);
                toc
                
                
                iters = 100;
                tic
                for i = 1:iters
                    [xout,fval,exitflag,iter,lambda,auxOutput] = qpOASES_sequence( 'm',QP,H,f,A,xl,xu,al,au,obj.qpParams.options);
                end
                with_hotstart = toc/iters
            end  
        end
        
    end
end

