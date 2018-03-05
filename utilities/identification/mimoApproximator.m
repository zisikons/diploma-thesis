classdef mimoApproximator
%mimoApproximator Class for regressor approximations in a MIMO system.
%
%   This class containts the necessary information for describing or 
%   approximating a MIMO system using any approximator. This is a template
%   class in the sense that it works for every approximator of the form
%   f_hat(x) = linear_weight * regressor_vector.
%
%   How to use:
%     # TODO
%
    properties
        % System related parameters, maybe even not necessary
        N
        M
        
        % Weights
        wf
        wg
    end
    
    methods
        % Structural information i guess, which functions are used, which
        % not, etc 
        
        % Constructor
        function obj = mimoApproximator( N )
            % MIMO Description
            obj.N = N(:);
            obj.M = size(obj.N,1);
        end
        
        % =============== To implement ===============
        % Function estimates for control loop
        Phi(obj,x);      % Ginv(x) * F (x)
        Gamma(obj,x);    % Ginv(x)
        
        % Update Laws
        estimatorDerivatives(obj, varargin);    % d(x_hat) / dt
        setWeights(obj, varargin);
        
        % Request Size for Functions
        
        % Maybe even more
    end
    
end

