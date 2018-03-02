function [ f ] = regressorGenerator( centers, sigmas, bias )
%REGRESSORGENERATOR Gaussian regressor function generator.
%
%  Usefull in RBF Neural Network identification applications. This function
%  provides a handler Z(x), given the centers and the variation of the
%  gaussian function of the network. Use this function providing the
%  centers columnwise. 
%
%  Warning: The function itself does not perform any checks regarding
%  the compatibility of x and the regressor centers.
%
%  Usage:
%    centers = [-1 -1 0 1 1;
%               -1  1 0 -1 1];
%
%    sigma = 0.45;
%    bias  = false; 
%    Z = regressorGenerator( centers, sigma, bias )
%    Z = regressorGenerator( centers, sigma ) % same, default bias = false
%
%    x = [0;0];
%    Z(x)       % centersNumber x 1 regressor vector 
%
%

    % Handle case where bias is requested
    if nargin == 2
      bias = false;
    end

    % Read Vector sizes
    centersDim = size(centers,1);
    centersN   = size(centers,2);
    sigmasN    = size(sigmas,1);
    
    % Case where only one sigma is given
    if  sigmasN~= 1
        error('Current implementation works only with one sigma')
        return
    end

    % Return Template
    f = @(x) regressor_template(x,centers,sigmas,bias);

end
function y = regressor_template(x,centers,sigmas,bias)

    % Vectorized norm calculation
    diffs = bsxfun(@minus,centers,x);
    total_norm = diffs.*diffs;
    total_norm = sqrt(sum(total_norm,1))';
    
    % Exponent and gaussian form
    y = exp(-(total_norm/sigmas).^2);
  
    if bias == true
      y = [1;y];
    end
end
