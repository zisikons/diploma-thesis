classdef SimplePPController <  AbstractController
%SimplePPController Simplest form of prescribed performance controller.
%   Detailed explanation goes here
    
    properties
        performance
        transform
        
        % Gains ( default ? )
        gains = struct('k',1,'lambda',1);
        
        % Auxilary variables 
        surfaceCoeff    % coefficients for each sub-system
        coeffMatrix     % merged version for easier in-loop
                        % calculations
    end
    
    methods
        % Basic Constructor
        function obj = SimplePPController(N, performance, transformation)
            % Initiallize base class
            obj = obj@AbstractController(N);
            
            % Store classes
            obj.performance = performance;
            obj.transform = transformation;
                        
            % Surface coefficients
            for i = 1:obj.M
                binomialCoeff = binomialCoefficients(N(i)-1);
                exponents = (N(i)-1):-1:0;
                obj.surfaceCoeff{i} = binomialCoeff.*(obj.gains.lambda.^exponents);
            end
            
            % This matrix is used for instant multiplication with the error
            % vector
            obj.coeffMatrix = [obj.surfaceCoeff{:}]';
        end
        
        function [u,ksi,e] = controlLaw(obj, t, x, x_ref)
            %% Create Signals
            e   = x - x_ref;
            
            % Create the surface error for each sub-system
            sigma = zeros(obj.M, size(x,2));
            tmp = bsxfun(@times,e,obj.coeffMatrix); % gucci
            
            % Calculate surface value for every sub-system
            subIndexes = [0;cumsum(obj.N)];
            for i = 1:obj.M
                sigma(i,:) = sum(tmp(1+subIndexes(i) : subIndexes(i+1),:),1);
            end
            
            % Ksi(t) for each time instance
            ksi = bsxfun(@rdivide,sigma,obj.performance.rho(t));
            
            %% Return u(t) using the transform
            u = obj.transform.T(ksi);
        end
        
        function controlSignals()
        end  
        
        function v = leftoverErrors(obj, t, x, x_r)
            %% Signals
            % Trajectory error
            e   = x - x_r;
            
            % Coefficients and error vector filtering
            % coefficients
            aux = cumsum(obj.N);
            tmp = obj.coeffMatrix;
            tmp(aux) = [];        % remove order states  coefficients
            
            % error vector filter
            aux = cumsum([1;obj.N(1:end-1)]);
            e(aux,:) = [];        % remove order states from error vector
            
            %% Final sum calculation (for every time step) 
            v = zeros(obj.M,size(x,2));
            tmp = bsxfun(@times,e,tmp);
            
            % Calculate leftovers for every sub-system
            subIndexes = [0;cumsum(obj.N - 1)];
            for i = 1:obj.M
                v(i,:) = sum(tmp(1+subIndexes(i) : subIndexes(i+1),:),1);
            end
            
        end
        
    end
    
end
function c = binomialCoefficients(order)
%%binomialCoefficients Calculate binomial coefficients up to order N

    k = 1:order;
    c = [1 cumprod((order-k+1)./k)];

end

