classdef PolynomialReference < ReferenceSystem
%PolynomialReference Class providing utility functions for describing a
%Polynomial reference system for MIMO systems.
%
%   A reference system is used to produce reference signals x_i(t) and  
%   u_r(t), thus creating a trajectory for a real system to follow. The term
%   polynomial refers to the assumption, that in this scenario the control
%   input u(t) is a polynomial of t, resulting in every other state
%   trajectory being also a polynomial of t. In order to use the polynomial
%   reference system, one must specify a starting point x0, an ending point
%   xT and the transition duration DT.
%
%   PolynomialReference(N,DT) instantiates a polynomial reference object of
%   class order N, and a transitions duration time DT. N can be a positive
%   integer, or a vector of positive integers, describing a MIMO system
%   with relative degrees.
%   
%   Usage:
%     % single input case
%     pRef = PolynomialReference(2,0.2);
%     x0 = [0;0];
%     xT = [0;1];
%
%     % MIMO case (subsytems of order 2 and 1 accordingly)
%     pRef = PolynomialReference([2,1],0.2);
%     x0 = [0;0;0];
%     xT = [0;1;2];
%
%     pRef.setTransition(x0, xT);  % parameterization for transition x0-xT
%     pRef.controlInput(t)         % u_r(t) at given time t 
%                                  % (see PolynomialReference/controInput) 
%     pRef.plant(t,x_r)            % plant form for system integration
%                                  % (see ReferenceSystem/plant)
%
    properties
        parameters
        x0
        xT
        DT
        t_start = 0
        
        inputBoundary = 0;
        inputCoeff           % polynomial coefficients for input calculation
        regressionMatrix
    end
    
    methods
        % Constructor
        function obj = PolynomialReference(N, DT)
            % Superclass constructor invocation
            obj = obj@ReferenceSystem(N);
            obj.DT = DT;     % transition time

            % Initiallize class internal variables
            obj.parameters = cell(obj.M,1);
            obj.inputCoeff = cell(obj.M,1);
            obj.regressionMatrix = cell(obj.M,1);

            % Calculate internal parameters
            obj.updateStaticParameters();
        end
        
        function updateStaticParameters(obj)
        %updateStaticParameters Calls updateStaticParametersSI(N(i)) for every
        %every subsystem.
            for i = 1:obj.M
                [obj.regressionMatrix{i}, ...
                 obj.inputCoeff{i}]= obj.updateStaticParametersSI(obj.N(i));                  
            end
        end
        
        function setInitTime(obj, t_start)
        %setInitTime Internal variable update for time transforms.
        %
        %  Polynomial reference class provides an internal variable t_start
        %  so the transformation [t0, t0 + DT] is done internally and is
        %  not left to the user. This function can be also called from
        %  setTransition( ) as an optional argument.
        
            obj.t_start = t_start;
        end
        
        % Implemented in other files (more complex functions)
        [regressionMatrix, inputCoeff] = updateStaticParametersSI(obj,N)
        u = controlInput(obj, t)
        setTransition(obj, x0, xT,t_start)        
    end
end

