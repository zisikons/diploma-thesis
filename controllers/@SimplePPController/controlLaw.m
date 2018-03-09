function [u,ksi,e] = controlLaw(obj, t, x, x_ref)
%%controlLaw Online and offline PPC control input calculation.
%
%   Given the system state x, the desired trajectory state x_ref and the
%   time stamp t, controller.controlLaw(t, x, x_ref) calculates the
%   prescribed performance controller input u(t).
%
%   Usage:
%     % inside control loop 
%     u = -controller.contolLaw(t, x, x_ref)
%
%     % after simulation
%     [T, X] = ode45( ... );
%     X_plant = X(:, plant_states)';
%     X_ref   = X(:, ref_states)';
%     [U, KSI, E] = controller.contolLaw(T, X_plant, X_ref);
%     % ^ control input, normallized error and state errors ^ 
%
    %% Create Signals
    e   = x - x_ref;

    % Create the surface error for each sub-system
    sigma = zeros(obj.M, size(x,2));
    tmp = bsxfun(@times,e,obj.coeffMatrix); % gucci

    % Calculate surface value for every sub-system
    subIndexes = [0;cumsum(obj.N)];
    for i = 1:obj.M
        sigma(i,:) = sum(tmp(1+subIndexes(i) : subIndexes(i+1),:),1);
    end

    % Ksi(t) for each time instance
    ksi = bsxfun(@rdivide,sigma,obj.performance.rho(t));

    %% Return u(t) using the transform
    u = obj.transform.T(ksi);

end
