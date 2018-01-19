classdef PPCTransformation
%PPCTransformation Class to describe PPC transformation and the associated functions.
%
%   The PPC Transformation in practice is a mathematical function used for
%   transforming an error signal e(t) from it's initial bounded domain (m,M)
%   to an unbounded signal eps(t) and vice versa. The use of this function,
%   it's inverse function and their derivatives is essential when designing
%   a Prescribed Performance Controller.
%
%   PPCTransformation() instantiates an object providing the necessary functions
%   used when applying the PPC methodology. The parameters used in the no-constructor 
%   case are the defaults (limits = [-1,1] and c = 1/2)
%
%   PPCTransformation(limits, scale) instantiates an object, parameterizing
%   it with the user-selected values of limits and scale parameters.
%
%   Initiallization:
%     % Default values
%     transform = PPCTransformation();
%
%     % User selected values
%     lim = [-2,2];
%     c   = 1;
%     transform = PPCTransformation(lim, c);
%
%   Methods:
%     eps  = transform.T(ksi);      % transformation to generallized error
%     ksi  = transform.invT(eps);   % inverse transofromation
%     sDot = transform.invTDot(eps) % inverse transofromation 1st derivative
%
%  (all methods work for vectorized inputs as well)    
    properties
        m = -1;
        M =  1;
        scale = 0.5;
    end
    
    methods
        % Constructor
        function [obj] = PPCTransformation(limits, scale)
            % If no argunment is given, use the defaults
            if nargin > 0
                obj.m = limits(1);
                obj.M = limits(2);
                obj.scale  = scale;
            end
        end
        
        % Straight Transformation (m,M) -> (-inf,inf)
        function [y] = T(obj, x)
            y = log((-obj.m + x)./(obj.M - x))...
                /(2 * obj.scale); 
        end
        
        % Inverse Transformation (-inf,inf) -> (m,M)
        function [x] = invT(obj, y)
            x = ( obj.M * exp(obj.scale*y) + obj.m * exp(-obj.scale*y) )...
                ./( exp(obj.scale*y) + exp(-obj.scale*y));
        end
        
        % Inverse Transformation Derivative
        function [y] = invTDot(obj, x)
            y = -2*obj.scale*exp(2*obj.scale*x)*(obj.m - obj.M)...
                ./((exp(2*obj.scale*x) + 1).^2);
        end
        
    end
    
end

