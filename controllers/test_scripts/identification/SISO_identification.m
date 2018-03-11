function [] = SISO_identification()
    %% Include utilities to path
    g = genpath('~/diploma-thesis/utilities/');
    addpath(g)
    
    g = genpath('~/diploma-thesis/controllers/');
    addpath(g)
    
    close all
    
    %% Approximator Description
    N = 2;
    
    % Phi(x) description
    centers = [0 0;
               1 0;
               0 1;
               1 1;]';

    f_parameters.centers  = centers;
    f_parameters.variance = 0.5;
    f_parameters.bias = false;

    % Gamma(x) description
    g_parameters.centers = [0 1];                
    g_parameters.variance = f_parameters.variance;
    g_parameters.active_states = [1];    % only a function of x(1)
    g_parameters.bias = true;

    % Approximator object initiallization
    approx2D = rbfApproximator( N, f_parameters, 'g_parameters', g_parameters );

    %% Identifier Initiallization
    % PPC Utilities
    per = PerfomanceFunction(2,2,0.01);
    trans = PPCTransformation();

    % Controller intiallization
    PPC_controller = PPCIdentifier(N, per, trans, approx2D);
    PPC_controller.gains.k = 10;
    
    % Gains Selection
    PPC_controller.approximator.weightGains.gaussianPhi   = 0.08;
    PPC_controller.approximator.weightGains.biasPhi       = 0.05;
        
    PPC_controller.approximator.weightGains.gaussianGamma = 2*0.06;
    PPC_controller.approximator.weightGains.biasGamma     = 1*0.05;
    
    % Reference system intiallization
    DT = 1;
    pRef = PolynomialReference(N, DT);
    
    % Centers to visit % already defined
    t_start = 0;
    T_sim = [0,DT];
   
    T_total = [];
    X_total = [];
    U_ref = [];
    
    systemSize = PPC_controller.approximator.approximatorSize() + 2*sum(PPC_controller.N);
    state = zeros(systemSize,1);
    
    %% Control loop
    travel_centers = [centers, [0;0]];
    
    tic
    for j = 1:300;
        for i = 1:size(travel_centers,2)-1
            
            % init
            %travel_centers
            xT = travel_centers(:,i+1);
            pRef.setTransition(state([3,4]),xT,t_start);

            % parameterize loop
            sim_loop = @(t,x) closed_loop(t,x,pRef,PPC_controller);

            % Simulate
            [T,X] = ode15s(sim_loop,T_sim,state);
            
            % Setup simulation parameters for next loop
            t_start = T(end);
            T_sim = T_sim + DT;
            state = X(end,:)';

            % Store results
            T_total = [T_total;T];
            X_total = [X_total;X];
            U_ref   = [U_ref; pRef.controlInput(T)'];

        end
        disp(['Loop = ',num2str(j)]);
    end
    toc

    figure()
    hold on
    plot(X_total(:,1),X_total(:,2))
    plot(X_total(:,3),X_total(:,4))
    plot(centers(1,:),centers(2,:))
    hold off
    
    figure()
    hold on
    grid on
    box on
    title('f(x) weights')
    plot(T_total, X_total(:,5:8))
    hold off
    
    figure()
    hold on
    grid on
    box on
    title('g(x) weights')
    plot(T_total, X_total(:,9:end))
    hold off

    save('SISO_identification.mat');
  
end
function dx = test_plant(t,x,u,controller)
    dx = zeros(2,1);
    
    f = [1 2 3 4] * controller.approximator.Zf(x);
    g = [0.5, 1, 2] * controller.approximator.Zg(x);
    
    % 1st subsystem
    dx(1) = x(2);
    dx(2) = f/g + (1/g)*u(1);


end
function dx = closed_loop(t,x,ref,controller)


    dx = zeros(size(x,1),1);
    % States
    x_plant = x(1:2);
    x_ref   = x(3:4);
    
    weights = x(5:end);
    controller.approximator.setWeights(weights);
    
    % Calculate
    u_ref = ref.controlInput(t);
    u = controller.controlLaw(t,x_plant,x_ref, u_ref);
    
    % Plant
    dx(1:2) = test_plant(t,x_plant,u,controller);
    
    % Reference Plant
    dx(3:4) = ref.plant(t,x_ref);
    dx(5:end) = controller.approximator.estimatorDerivatives(x_plant,controller.innovationTerms);
end
