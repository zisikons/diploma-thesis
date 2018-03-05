function proper_format =  validateStructFormat(obj,rbf_struct)

    % Check if "centers" is present
    if ( ~isfield(rbf_struct,'centers') )
        error(['Could not initiallize rbfApproximator object because ',...
               'field "centers" is missing from grid parameters struct. '])
    end

    % Check if "variance" is present
    if ( ~isfield(rbf_struct,'variance') )
        error(['Could not initiallize rbfApproximator object because ',...
               'field "variance" is missing from grid parameters struct. '])
    end

    % Check if there is a bias field and replace with the default
    % value
    if ( ~isfield(rbf_struct,'bias') )
        rbf_struct.bias = false;
    end

    % Check if there is an active states description
    if ( ~isfield(rbf_struct,'active_states') )

        if (size(rbf_struct.centers,1) ~= sum(obj.N))
           error(['Centers dimension smaller than system dimension. Specify ',...
                  'active states for clarification.']) 
        end


        % Check compatibility first
        rbf_struct.active_states = 1:sum(obj.N);
    end

    proper_format = rbf_struct;

end