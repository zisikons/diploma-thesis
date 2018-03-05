classdef  rbfApproximator < mimoApproximator
%rbfFunctoin Radial Basis Approximators for MIMO systems.
%
%   # TODO
%
%
    properties
  
        % Regressor related
        Zf    % Phi approximator
        Zg    % Gamma approximator
        
        % Regressor Parameters
        f_parameters
        g_parameters
    
        regressorSize_f
        regressorSize_g
    end
    
    methods
        % Structural information i guess, which functions are used, which
        % not, etc
        
        % Constructor
        function obj = rbfApproximator( N, f_parameters, varargin )
            
            % Include utilities to path
            g = genpath('~/diploma-thesis/utilities/identification/');
            addpath(g)
            
            % Input Parser to manage input cases
            p = inputParser;

            % Default Arguments
            addRequired(p,'N',@isvector)
            addRequired(p,'f_parameters',@isstruct);
            
            % Optional Arguments
            addParameter(p, 'g_parameters', f_parameters);
            
            % Idea for future: Enable different regressor function as an
            % initiallization, for future use of the code
            parse(p,N, f_parameters, varargin{:})
            
            % call super constructor to adjust system-order internal vars
            obj = obj@mimoApproximator(N);
            
            obj.f_parameters = validateStructFormat(obj,p.Results.f_parameters);
            obj.g_parameters = validateStructFormat(obj,p.Results.g_parameters);
            

            % Initiallize weight matrices
            obj.Zf = regressorGenerator(obj.f_parameters.centers,...
                                        obj.f_parameters.variance,...
                                        obj.f_parameters.bias);
                                
            obj.Zg = regressorGenerator(obj.g_parameters.centers,...
                                        obj.g_parameters.variance,...
                                        obj.g_parameters.bias);

            % Define wf and wg for this example
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
        
        % Function estimates for control loop
        function [phi] = Phi(obj,x)
            
        end
            % Ginv(x) * F (x)
        function [] = Gamma(obj,x)
            
        end
          
        function [] =  estimatorDerivatives(obj, varargin)
            
        end
        
        function [] =  setWeights(obj, varargin)
            
        end
        
        function proper_format =  validateStructFormat(obj,rbf_struct)
           
            % Check if "centers" is present
            if ( ~isfield(rbf_struct,'centers') )
                error(['Could not initiallize rbfApproximator object because ',...
                       'field "centers" is missing from grid parameters struct. '])
            end
            
            % Check if "variance" is present
            if ( ~isfield(rbf_struct,'variance') )
                error(['Could not initiallize rbfApproximator object because ',...
                       'field "variance" is missing from grid parameters struct. '])
            end
            
            % Check if there is a bias field and replace with the default
            % value
            if ( ~isfield(rbf_struct,'bias') )
                rbf_struct.bias = false;
            end
            
            % Check if there is an active states description
            if ( ~isfield(rbf_struct,'active_states') )
                
                if (size(rbf_struct.centers,1) ~= sum(obj.N))
                   error(['Centers dimension smaller than system dimension. Specify ',...
                          'active states for clarification.']) 
                end
                    
                    
                % Check compatibility first
                rbf_struct.active_states = 1:sum(obj.N);
            end
            
            proper_format = rbf_struct;
            
        end
        

    end
    
end

