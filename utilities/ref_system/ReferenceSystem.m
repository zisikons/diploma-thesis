classdef ReferenceSystem < handle
%ReferenceSystem  Interface Class for MIMO reference systems.
%
%   A reference system is used to produce reference signals x_i(t) and  
%   u_r(t), thus creating a trajectory for a real system to follow. This
%   class provides a template for reference systems designed for MIMO
%   systems in the canoncial form.
%   
%   Usage:
%     % construct base object inside the derived class
%     obj = obj@ReferenceSystem(N);
%

    properties
        N     % class order matrix
        M     % number of subsystems (maybe a more descriptive name needed)
    end
    
    methods

        function obj = ReferenceSystem(N)
            % Input verification
            if (~isvector(N))
               error('N must be a vector of positive integers.') 
            end

            % Store N array as a column vector
            obj.M = length(N);
            obj.N = N(:);
        end
        
        function dx = plant(obj,t, x)
        %%Plant Differential equation form of MIMO system at (t,x)
        %
        %  This function is used to provide signals x_i(t) inside an ode
        %  integration. The reference system is integrated alongside the 
        %  actual sytem and plant(obj,t, x) provides the differential
        %  equations of the reference system.
        %
        %  Usage:
        %    % integrate the reference system alone
        %    ref = RandomReference(N,varargin);
        %    [T,X] = ode45(@ref.plant, t_sim, x0);
        %
        %    % integrate inside a controller
        %    x_real = x(1:N);
        %    x_ref   = x(N+1:2*N);
        %
        %    ...
        %    dx(1:N)     = real_plant(t,x_real,u);
        %    dx(N+1:2*N) = ref.plant(t,x_ref);

            % Initiallization
            dx = ones(sum(obj.N), 1);

            % Bank of integrators
            dx(1:end-1) = x(2:end);

            % Assign control inputs
            inputIndexes = cumsum(obj.N);
            dx(inputIndexes) = obj.controlInput(t);
        end

    end
    
    methods (Abstract)
        % Control input vector for the reference system
        controlInput(obj, varargin)
    end
    
end

