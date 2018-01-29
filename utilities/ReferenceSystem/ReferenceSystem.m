classdef ReferenceSystem < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        N % class order
    end
    
    methods
        % Constructor
        function obj = ReferenceSystem(N)
            obj.N = N;   % order
        end
        
        % Plant Stracture Function
        function dx = plant(obj,t, x)
            dx = ones(obj.N, 1);
            
            % System
            dx(1:end-1) = x(2:end);
            dx(end)     = obj.controlInput(t);   % or maybe control input of t?
        end
    end
    
    methods (Abstract)
        % Control Input for reference system
        controlInput(obj, t)
        
        % Parameters for reference system
        calculateParameters(obj, varargin)
        
        % Get the control input for the hole vector ? (X, T)
        offlineInput(obj, varargin)
        
    end
    
end

