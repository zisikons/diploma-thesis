classdef PolynomialReference < ReferenceSystem
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        parameters
        x0
        xT
        DT
        
        inputBoundary = 0;
        inputCoeff         % polynomial coefficients for input calculation
        regressionMatrix
    end
    
    methods  
        function obj = PolynomialReference(N, DT)
            % Superclass constructor invocation
            obj = obj@ReferenceSystem(N);
            obj.DT = DT;     % transition time
            
            % Calculate internal parameters
            obj.updateStaticParameters();

        end
        
        function u = controlInput(obj, t)
            exponents = (obj.N+1):-1:0;
            u = t.^exponents * (obj.inputCoeff .* obj.parameters(1:(obj.N+2)));
        end
        
        function updateStaticParameters(obj)
        %UPDATESTATICPARAMETERS Utility function to calculate internal class
        %parameters after N (order) and DT (transition time) selection.
        %
        % Depending only on the transition time DT and the system order N,
        % every reference system of this type we can define a matrix R
        % which is needed for calculating the model free parameters for a
        % selected transition from a point x0 to a point xT. This function
        % calculates these static attributes and assigns them to class
        % variables for further calculations.
        
            N  = obj.N;
            
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
            obj.regressionMatrix = [coefficients .* 0.^exponents;...    % t start (assumed 0)
                                    coefficients .* obj.DT.^exponents]; % t end (DT)
            obj.inputCoeff = polyCoeff';
        end
        
        function calculateParameters(obj, x0,xT)
            % Update internal variables
            obj.x0 = x0;
            obj.xT = xT;
            
            % Transition parameters
            Y = [x0(:); obj.inputBoundary; xT(:); obj.inputBoundary;];
            obj.parameters = obj.regressionMatrix\Y;
        end
        
        function offlineInput(obj, varargin)
            
        end
        
    end
    
end

