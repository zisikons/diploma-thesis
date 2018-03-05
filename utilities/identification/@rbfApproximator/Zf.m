function [ y ] = Zf( obj, X )
%Zf Phi regressor wrapper for proper input handling.
%   
%   This wrappers attempts to handle input, identify the case and apply
%   the regressor function properly. To be more precise, the cases are
%   handled are: 
%
%   1) X is a matrix instead of a vector, the function treats each column
%   as a seperate input and returns a matrix containing the regressor
%   vectors for each column in X.
%
%   2) If regressor is a function of a subspace of the full system
%   dimension, this functions makes sure to pass the proper states to
%   regressor function
%
%   3) If there are any dimension missmatches, it prints the appropriate
%   error message ? Temporary disabled, but maybe will be reused
%
%   Usage:
%     % 2D-system (will see)
%     
%
%

    % Maybe add as internal variable to reduce calculations in closed loop
    % ?
    y = zeros(obj.regressorSize_f, size(X,2));

    for i = 1:size(X,2)
        y(:,i) = obj.regressorPhi( X(obj.f_parameters.active_states ,i) );   
    end


end

