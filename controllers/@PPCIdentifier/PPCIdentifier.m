classdef PPCIdentifier < SimplePPController
%PPCIdentifier Identification scheme based on prescribed performance
%cotnroller.
%
%   How to use:
%   Detailed explanation goes here
    
    properties
        approximator
        innovationTerms

    end
    
    methods
        % Constructor
        function obj = PPCIdentifier(N, performance, transformation, approximator)
            % Call PPC constructor
            obj = obj@SimplePPController(N, performance, transformation);
            
            % Copy handle to approximator object
            obj.approximator = approximator;
        end
        
        % Control Law - online version
        function [u,ksi,e] = controlLaw(obj, t, x, x_ref, u_ref)
            % Innovation term
            v = -u_ref + obj.leftoverErrors(t, x, x_ref);
            
            % It works
            [u,ksi,e] = controlLaw@SimplePPController(obj,t,x,x_ref);
            
            u = u - obj.approximator.Phi(x) ...
                  - obj.approximator.Gamma(x) * (v - ksi*obj.performance.rhoDot(t));
            
            % Update Innovation Terms
            obj.innovationTerms.Phi   = ksi./obj.performance.rho(t);
            obj.innovationTerms.Gamma = ksi./obj.performance.rho(t) * (v - ksi*obj.performance.rhoDot(t))' ;
        end
        

        
        function innovationTermsOffline(obj)
        end
        %%%%% Methods to implement:
        % 1) control input offline
        % 2) innovation terms offline
        
    end
    
end

