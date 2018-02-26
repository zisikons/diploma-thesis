function setTransition(obj, x0, xT, t_start)
%setTransition Set parameters for the transition x0 -> xT
%
% Calculates and sets the system's free parameters in order to
% match the boundary conditions for a x0 to xT transition. When used for
% different time 
% 
% Usage:
%   x0 = []; % column vector of system total size
%   xT = []; % column vector of system total size
%   
%   t_start = 10;
%   pRef.setTransition(x0,xT);
%   pRef.setTransition(x0,xT,t_start); % polynomial reference class handles
%                                      % internally the transformation
%                                      % from [10, 10 + DT] to [0,DT]
    %% If t_start is provided as an argument, update the internal variable
    if (nargin > 3)
        obj.setInitTime(t_start);
    end
    
    %% Update internal variables
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

