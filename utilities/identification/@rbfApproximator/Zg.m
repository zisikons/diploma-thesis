function [ y ] = Zg( obj, X )
%Zg Phi regressor wrapper for proper input handling.
%   
%   Read the description of Zf, exaclty the same for g regressors.

    % Maybe add as internal variable to reduce calculations in closed loop
    % ?
    y = zeros(obj.regressorSize_g, size(X,2));

    for i = 1:size(X,2)
        y(:,i) = obj.regressorGamma( X(obj.g_parameters.active_states ,i) );   
    end


end

