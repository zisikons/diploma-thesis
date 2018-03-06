clear
%% Case 1: Simple 2 order system, euler
N = 2;
centers = [0 0;
           0 1;
           1 0;
           1 1]';

f_parameters.centers  = centers;
f_parameters.variance = 0.5;
f_parameters.bias = true;

g_parameters.centers = [0 1]; % exei ginei malakia
                              % parolo poy den exo aktive states to pernaei
g_parameters.variance = 0.3;
g_parameters.active_states = 1;

approx2D = rbfApproximator( N, f_parameters, 'g_parameters',g_parameters );
approx2D.setWeights( (1:approx2D.approximatorSize())' );
disp(['Reduced g(x) 2D,size = ' ,num2str(approx2D.approximatorSize())])


%% Case 2: Simple 2 order system, g(x) = full state
approx2DFull = rbfApproximator( N, f_parameters);
disp(['g(x) full state, size = ' ,num2str(approx2DFull.approximatorSize())])

%% Case 3: MIMO [2,1], redused
clear f_parameters g_parameters
N = [2,1];
centers = [0 0 0;
           0 0 1;
           0 1 0;
           0 1 1;
           1 0 0;
           1 0 1;
           1 1 0;
           1 1 1;]';

f_parameters.centers  = centers;
f_parameters.variance = 1;
f_parameters.bias = false;

g_parameters.centers = [0 0;
                        0 1;
                        1 0;
                        1 1]';
g_parameters.variance = 1;
g_parameters.active_states = [1,3];
g_parameters.bias = false;

approx3D = rbfApproximator( N, f_parameters, 'g_parameters',g_parameters );
disp(['Reduced g(x) 3D, size = ' ,num2str(approx3D.approximatorSize())])

approx3D.setWeights( (1:approx3D.approximatorSize())' );
