classdef PerfomanceFunction
%PerfomanceFunction Class to describe PPC performance functions.
%
%   Performance functions are used in PPC control methodology in
%   order to describe a desired transient response. Each function is 
%   described using a convergence rate, a initial value and a steady
%   state value.
%
%   PerfomanceFunction(expDecay, r0, rInf) instantiates a performance
%   function object with expDecay convergence rate, r0 initial
%   value and rInf steady state value
%
%
%   Usage: 
%   performance = PerfomanceFunction(expDecay, r0, rInf);
%
%   % scalar case
%   performance.rho(t)    % value of the performance function at time t
%   performance.rhoDot(t) % value of the performance function's first
%                         % derivative at time t
%
%   % vector case
%   performance.rho(T)    % matrix of the performance function values
%                         % for each instance of matrix T
    
    properties     % To add validation in new Matlab version
        expDecay
        r0
        rInf
    end
    
    methods
        % Constructor
        function [obj] = PerfomanceFunction(expDecay, r0, rInf)
            obj.expDecay = expDecay;
            obj.r0 = r0;
            obj.rInf = rInf;
        end
        
        % performance function at time t
        function [y] = rho(obj, t)
            y = (obj.r0 - obj.rInf)*exp(-obj.expDecay*t) + obj.rInf;
        end
        
        % performance function 1st derivative at time t
        function [y] = rhoDot(obj, t)
            y = -obj.expDecay*(obj.r0 - obj.rInf)*exp(-obj.expDecay*t);
        end
    end

end

