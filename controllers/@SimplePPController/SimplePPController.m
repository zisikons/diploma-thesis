classdef SimplePPController <  AbstractController
%SimplePPController Simplest form of prescribed performance controller.
%
%   The prescribed performance control is a model-agnostic control
%   methodology which allows one to perform trajectory tracking with the
%   surface absolut error |s(t)| remaining bounded from a user-selected
%   performance function r(t). For more info see:
%   http://ieeexplore.ieee.org/document/4639441/
%
%   SimplePPController(N, performance, transformation) instantiates a
%   controller object for systems of class order N (can be of the form [N1,
%   N2, ...] for MIMO systems). The transient response of the error is
%   described by the performance object which is of type
%   PerformanceFunction, and the functions used for calculating the control
%   input are provided by the transformation object, of type
%   PPCTransformation.
%
%   Basic Usage:
%     % PPC Utilities
%     per = PerfomanceFunction(2,2,0.01);  % perfomance function
%     trans = PPCTransformation();         % default tranformation
%
%     % Controller object initiallization
%     orders = [2,1];
%     PPC_controller = SimplePPController(orders,per,trans);
%
%     % Set gains (#TODO)
%
%     % Control input
%     u = -PPC_controller.controlLaw(t,x_plant,x_ref);     % in closed loop
%     U = PPC_controller.controlLaw(T', X_plant', X_ref'); % signals after sim 

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
        
    end
    
end
function c = binomialCoefficients(order)
%%binomialCoefficients Calculate binomial coefficients up to order N

    k = 1:order;
    c = [1 cumprod((order-k+1)./k)];

end
