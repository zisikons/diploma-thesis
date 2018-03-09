function w_hat_dot =  estimatorDerivatives(obj, x, innovationTerms, gains)
%%estimatorDerivatives Update Laws for RBF Approximator.
%
%   When using the approximator in a closed loop for identification
%   purposes, use this function to provide the closed loop with the
%   approximator weights derivatives. As this class is designed to describe
%   an approximator, regardless of it's participation in an Identification
%   schema, it does not possess any information regarding the identification
%   gains or the innovation terms, so they are passed as arguments.
%
%   How to use:
%       - Pass innovationTerms as a struct containing two terms, Phi and
%       Gamma. Phi is a column vector containing innovation terms for phi1,
%       phi2, ... respectively. Gamma terms are passed as a vecotr
%       containing terms [gamma11, gamma21, ..., gammaM1, gamma12, gamma22,
%       ..., gamma2M, ...] respectively.
%
%       -  gains must be a struct containing the terms: gaussianPhi,
%       gaussianGamma, and if biases are used biasPhi and biasGamma.
%
%   Tests to be done:
%       2nd order system [2] without biases | check
%       2nd order system [2] with    biases | check
%       3rd order system [2,1] without biases | not yet
%       3rd order system [2,1] with    biases | not yet

    %%%%% Signals Generation
    % Phi
    wf_dot = repmat(obj.Zf(x), 1, obj.M);
    wf_dot = bsxfun(@times, wf_dot , innovationTerms.Phi');
    
    % Gamma
    wg_dot = repmat(obj.Zg(x), 1, obj.M^2);
    wg_dot = bsxfun(@times, wg_dot, innovationTerms.Gamma(:)');
    
    %%%%% Apply Gains
    % Gaussian
    wf_dot((1 + obj.f_parameters.bias):end, :) =     gains.gaussianPhi * wf_dot((1 + obj.f_parameters.bias):end, :);
    wg_dot((1 + obj.g_parameters.bias):end, :) =   gains.gaussianGamma * wg_dot((1 + obj.g_parameters.bias):end, :);
    
    % Bias
    if obj.f_parameters.bias
      wf_dot(1) =   gains.biasPhi * wf_dot(1);
    end

    if obj.g_parameters.bias
      wg_dot(1) =   gains.biasGamma * wg_dot(1);
    end

    %%%%% Return results, in the input form
    w_hat_dot = [wf_dot(:);wg_dot(:)];

            
end
