classdef  rbfApproximator < mimoApproximator
%rbfFunctoin Radial Basis Approximators for MIMO systems.
%
%   # TODO
%       % Idea for future: Enable different regressor function as an
%       % initiallization, for future use of the code
%
%   # Notes:
%      I think current imlementation of weights and storage is extremely
%      inefficient because of the extra transposes requiered
    properties
  
        %%%%% Regressor related
        % functions
        regressorPhi    % Phi approximator
        regressorGamma  % Gamma approximator
        
        % parameters
        f_parameters
        g_parameters
    
        % size
        regressorSize_f
        regressorSize_g
    end
    
    methods        
        % Constructor method : Setups the class
        function obj = rbfApproximator( N, f_parameters, varargin )
            
            %%%%%%%%%%%% Parse Arguments
            %Include utilities to path
            g = genpath('~/diploma-thesis/utilities/identification/');
            addpath(g)
            
            % Input Parser to manage input cases
            p = inputParser;

            % Default Arguments
            addRequired(p,'N',@isvector)
            addRequired(p,'f_parameters',@isstruct);
            
            % Optional Arguments
            addParameter(p, 'g_parameters', f_parameters);
            parse(p,N, f_parameters, varargin{:})
            
            % call super constructor to adjust system-order internal vars
            obj = obj@mimoApproximator(N);
            
            obj.f_parameters = validateStructFormat(obj,p.Results.f_parameters);
            obj.g_parameters = validateStructFormat(obj,p.Results.g_parameters);
            
            
            %%%%%%%%%%%% Initiallize class-internal parameters 
            % Regressors
            obj.regressorPhi   = regressorGenerator(obj.f_parameters.centers,...
                                                    obj.f_parameters.variance,...
                                                    obj.f_parameters.bias);
                                
            obj.regressorGamma = regressorGenerator(obj.g_parameters.centers,...
                                                    obj.g_parameters.variance,...
                                                    obj.g_parameters.bias);

            % Sizes
            obj.regressorSize_f = size(obj.f_parameters.centers,2) + obj.f_parameters.bias;
            obj.regressorSize_g = size(obj.g_parameters.centers,2) + obj.g_parameters.bias;
            
            % Preallocate Weights
            obj.wf = zeros(obj.M, obj.regressorSize_f);
            obj.wg = zeros(obj.M^2, obj.regressorSize_g);
            
        end
        
        % =============== To implement ===============
        function s = approximatorSize(obj)
           s =  obj.M * obj.regressorSize_f + ...  % f part
                obj.M^2 * obj.regressorSize_g;     % g_part
        end
        
        %%%%%%%%%%% Function estimates for control loop
        function phi = Phi(obj,x)
            % Returns in the form: 
            %
            % phi(:,1) = phi(t0)
            % phi(:,2) = phi(t1)
            %         ...
             phi = obj.wf * obj.Zf(x);
        end
        
        % Ginv(x) * F (x)
        function gamma = Gamma(obj,x)
            % Returns in the form: 
            %
            % gamma(:,:,1) = G(t0)
            % gamma(:,:,2) = G(t1)
            %        ...
            gamma = obj.wg * obj.Zg( x );
            gamma = reshape(gamma, obj.M ,obj.M ,[]);
            
        end
        
        %%%%%%%%%%% Signatures defined in files
        proper_format =  validateStructFormat(obj,rbf_struct);
        y  = Zf( obj, X );
        y  = Zg( obj, X );
        
        setWeights(obj, w_hat);
        varargout =  estimatorDerivatives(obj, varargin)

    end
    
end

