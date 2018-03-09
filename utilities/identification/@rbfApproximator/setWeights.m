function [] =  setWeights(obj, w_hat)
% Naive implementation. There is a tryhard alternative
%
%   current storage = [ --- wf1 weights ----, ---- wf2 weights ----, ...
%                       --- wg11 weights ---, ----wg21 weights ----,...
%                       --- wg21 weights ---, ----wg22 weights ----,...
%                                            ... ]
%   but I think this is very slow in closed loop.

    % Defines who the weights are stored;
    obj.wf = reshape(w_hat(1:(obj.M * obj.regressorSize_f)),...     % reshape
                     obj.regressorSize_f, obj.M)';                  % the f part

    obj.wg = reshape(w_hat(1+(obj.M * obj.regressorSize_f):end),... % reshape
                     obj.regressorSize_g, obj.M^2)';                % the g part
            
end