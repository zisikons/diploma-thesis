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