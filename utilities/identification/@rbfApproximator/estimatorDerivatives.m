function w_hat_dot =  estimatorDerivatives(obj, x, innovationTerms )
%%estimatorDerivatives Update Laws for RBF Approximator.
%
%   When using the approximator in a closed loop for identification
%   purposes, use this function to provide the closed loop with the
%   approximator weights derivatives. As this class is designed to describe
%   an approximator, regardless of it's participation in an Identification
%   schema, it does not possess any information regarding the innovation 
%   terms so they are passed as arguments.
%
%   How to use:
%       - Pass innovationTerms as a struct containing two terms, Phi and
%       Gamma. Phi is a column vector containing innovation terms for phi1,
%       phi2, ... respectively. Gamma terms are passed as a vecotr
%       containing terms [gamma11, gamma21, ..., gammaM1, gamma12, gamma22,
%       ..., gamma2M, ...] respectively.
%
%       Gains handling:
%       -  Weight gains are calculated using 2 stuctus. weightGains must be a 
%       struct containing the terms: gaussianPhi,gaussianGamma, and if biases 
%       are used biasPhi and biasGamma, and practicaly is a way of
%       controlling convergance rate in each case ( Gamma or Phi and bias
%       or gaussian ). The second struct is named functionGains and is used
%       to control the convergance rate gains of a weight group refering to
%       a function. (for example gamma{11} gains ).
%
%   Tests to be done:
%       2nd order system [2] without biases   | check
%       2nd order system [2] with    biases   | check
%       3rd order system [2,1] without biases | check
%       3rd order system [2,1] with    biases | check

    %%%%% Signals Generation
    % Phi
    wf_dot = repmat(obj.Zf(x), 1, obj.M);
    wf_dot = bsxfun(@times, wf_dot , innovationTerms.Phi');
    
    % Gamma
    wg_dot = repmat(obj.Zg(x), 1, obj.M^2);
    wg_dot = bsxfun(@times, wg_dot, innovationTerms.Gamma(:)');
    
    %%%%% Apply Gains
    % Gaussian
    wf_dot((1 + obj.f_parameters.bias):end, :) = obj.weightGains.gaussianPhi * wf_dot((1 + obj.f_parameters.bias):end, :);
    wg_dot((1 + obj.g_parameters.bias):end, :) = obj.weightGains.gaussianGamma * wg_dot((1 + obj.g_parameters.bias):end, :);
    
    % Bias
    if obj.f_parameters.bias
        wf_dot(1,:) = obj.weightGains.biasPhi * wf_dot(1,:);
    end

    if obj.g_parameters.bias
        wg_dot(1,:) = obj.weightGains.biasGamma * wg_dot(1,:);
    end
    
    wf_dot = bsxfun(@times,wf_dot,obj.functionGains.Phi);
    wg_dot = bsxfun(@times,wg_dot,obj.functionGains.Gamma);
    
    %%%%% Return results, in the input form
    w_hat_dot = [wf_dot(:);wg_dot(:)];

            
end
