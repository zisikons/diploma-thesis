classdef AbstractController  % Inherit from Handle?
%AbstractController Interface class for controllers
%
%   Detailed explanation goes here
    
    properties
        N     % system stracture
        M     % number of subsystems
        
    end
    
    methods
        % Base Constructor
        function obj = AbstractController(N)
            % Input verification
            if (~isvector(N))
               error('N must be a vector of positive integers.') 
            end

            % Store N array as a column vector
            obj.M = length(N);
            obj.N = N(:);

        end
        
        % To implement
        controlSignals(obj,varargin);
        controlLaw(obj,varargin);
    end
    
end

