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
        
        function [regressionMatrix, inputCoeff] = updateStaticParametersSI(obj,N)
        %updateStaticParametersSI Utility function to calculate a single subsystem's  
        %internal parameters.
        %
        % Depending only on the transition time DT and the sub-system order N,
        % for every reference system of this type we can define a matrix R
        % and a coefficents vector inputCoeff. Both of these attributes are
        % needed for further calculations such as control input calculation
        % and subsystem parameter vector calculation (solution of R*x = Y).
        % This function is used in the constructor, and users normally should 
        % never need to call it.
        %
        % Usage:
        %   n = N(1);  % 1st substem order
        %   obj.updateStaticParametersSI(n) % (after DT is specified)
            
            % Regressor Matix Calculation
            coefficients = zeros(N + 1, 2*(N + 1));
            exponents    = zeros(N + 1, 2*(N + 1));
            
            % Polynomial coefficients of the first state
            polyCoeff = ones(1, 2*(N + 1));     % tmp variable
            coefficients(1,:) = ones(1, 2*(N + 1));
            exponents(1,:)    = (2*N+1):-1:0;
            
            % Derivatives
            for i = 2:(N+1)
                polyCoeff = polyder(polyCoeff); 
                
                coefficients(i,:) = [polyCoeff, zeros(1,i-1)];
                exponents(i,:)    = [exponents(i-1,2:end), 0];
            end
            
            % Storage
            regressionMatrix = [coefficients .* 0.^exponents;...    % t start (assumed 0)
                                coefficients .* obj.DT.^exponents]; % t end (DT)
            inputCoeff = polyCoeff';
        end
        
        function u = controlInput(obj, t)
        %controlInput Calculate the control input for a parameterized
        %system at time t, or for a time vector T.
        %
        % This function uses the internal variable parameters, so make sure
        % it is called after a ref.setTransition(x0,xT) call.
        %
        % Usage:
        %   %  Time instant t
        %   u = pref.controlInput(t);
        %   u = [u1(t)
        %        u2(t)
        %         ...
        %        uM(t)]
        %
        %   % Time vector T
        %   T = 0:0.1:DT;
        %   u = pref.controlInput(T);
        %   u = [ ----- u1(T) -----
        %         ----- u1(T) -----
        %                ...
        %         ----- uM(T) -----]
        %
            % Initiallization
            t = t(:);
            u = zeros(obj.M, length(t));
            
            % Calculate input vector for each subsystem
            for i = 1:obj.M
                exponents = (obj.N(i)+1):-1:0;
                T = bsxfun(@power,t,exponents); % vectorized form for offline calculations
                modelParameters = obj.parameters{i};
                
                % ith-subsystem input vector
                uV    = T * (obj.inputCoeff{i} .* modelParameters(1:(obj.N(i)+2)));
                u(i,:) = uV';
            end
        end
        
        function setTransition(obj, x0, xT)
        %setTransition Set parameters for the transition x0 -> xT
        %
        % Calculates and sets the system's free parameters in order to
        % match the boundary conditions for a x0 to xT transition. 
        % 
        % Usage:
        %   x0 = []; % column vector of system total size
        %   xT = []; % column vector of system total size
        %   pRef.setTransition(x0,xT);

            % Update internal variables
            obj.x0 = x0;
            obj.xT = xT;
            
            % Auxilary matrix for partitioning input to subsystem states
            subIndexes = [0;cumsum(obj.N)];
            
            % Calculate model parameters for each subsystem
            for i = 1:obj.M
                Y = [x0(1+subIndexes(i) : subIndexes(i+1));
                     obj.inputBoundary;
                     xT(1+subIndexes(i) : subIndexes(i+1));
                     obj.inputBoundary;];
                
                % Rx = Y system solution
                obj.parameters{i} = obj.regressionMatrix{i}\Y;
            end
        end
    end
    
end

