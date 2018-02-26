function v = leftoverErrors(obj, t, x, x_r)
%%leftoverErrors Leftover error terms in PPC methodology.
%
%   Leftover terms are used when applying the PPC methodology alongside
%   an identification schema (and not only I imagine).
%
%   Usage:
%     % inside control loop 
%     u = -controller.leftoverErrors(t, x, x_ref)
%
%     % after simulation
%     [T, X] = ode45( ... );
%     X_plant = X(:, plant_states)';
%     X_ref   = X(:, ref_states)';
%     V = controller.contolLaw(T, X_plant, X_ref);
%
    %% Signals
    % Trajectory error
    e   = x - x_r;

    % Coefficients and error vector filtering
    % coefficients
    aux = cumsum(obj.N);
    tmp = obj.coeffMatrix;
    tmp(aux) = [];        % remove order states  coefficients

    % error vector filter
    aux = cumsum([1;obj.N(1:end-1)]);
    e(aux,:) = [];        % remove order states from error vector

    %% Final sum calculation (for every time step) 
    v = zeros(obj.M,size(x,2));
    tmp = bsxfun(@times,e,tmp);

    % Calculate leftovers for every sub-system
    subIndexes = [0;cumsum(obj.N - 1)];
    for i = 1:obj.M
        v(i,:) = sum(tmp(1+subIndexes(i) : subIndexes(i+1),:),1);
    end

end