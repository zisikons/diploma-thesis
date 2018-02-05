function setTransition(obj, x0, xT)
%setTransition Set parameters for the transition x0 -> xT
%
% Calculates and sets the system's free parameters in order to
% match the boundary conditions for a x0 to xT transition. 
% 
% Usage:
%   x0 = []; % column vector of system total size
%   xT = []; % column vector of system total size
%   pRef.setTransition(x0,xT);

    % Update internal variables
    obj.x0 = x0;
    obj.xT = xT;

    % Auxilary matrix for partitioning input to subsystem states
    subIndexes = [0;cumsum(obj.N)];

    % Calculate model parameters for each subsystem
    for i = 1:obj.M
        Y = [x0(1+subIndexes(i) : subIndexes(i+1));
             obj.inputBoundary;
             xT(1+subIndexes(i) : subIndexes(i+1));
             obj.inputBoundary;];

        % Rx = Y system solution
        obj.parameters{i} = obj.regressionMatrix{i}\Y;
    end
end

