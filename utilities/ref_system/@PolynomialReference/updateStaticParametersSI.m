function [regressionMatrix, inputCoeff] = updateStaticParametersSI(obj,N)
%updateStaticParametersSI Utility function to calculate a single subsystem's  
%internal parameters (SI refers to single input).
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