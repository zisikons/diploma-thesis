function [  ] = MIMO_identification(  )
    %% Include utilities to path
    g = genpath('~/diploma-thesis/utilities/');
    addpath(g)
    
    g = genpath('~/diploma-thesis/controllers/');
    addpath(g)
    
    close all
    
    %% Approximator Description
    N = [2, 1];
    
    % Phi(x) description
    centers = [0 0 0;
               1 0.5 0.3;
               0.8 0.7 0.5;]';

    f_parameters.centers  = centers;
    f_parameters.variance = 0.5;
    f_parameters.bias = true;

    % Gamma(x) description / same as F

    % Approximator object initiallization
    approx3D = rbfApproximator( N, f_parameters );

    %% Identifier Initiallization
    % PPC Utilities
    per = PerfomanceFunction(2,2,0.01);
    trans = PPCTransformation();

    % Controller intiallization
    PPC_controller = PPCIdentifier(N, per, trans, approx3D);
    PPC_controller.gains.k = 20;
    
    % Gains Selection
    PPC_controller.approximator.weightGains.gaussianPhi   = 0.15;
    PPC_controller.approximator.weightGains.gaussianGamma = 0.4;
    PPC_controller.approximator.weightGains.biasPhi       = 0.01;
    PPC_controller.approximator.weightGains.biasGamma     = 0.008;
    
    PPC_controller.approximator.functionGains.Phi =  [1,1]; % [f1,f2]
    PPC_controller.approximator.functionGains.Gamma = [1,0.6,1,6]; 
                                                     % [g11,g21,g12,g22]
    
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
    travel_centers = [centers, [0;0;0]];
    
    tic
    for j = 1:10; % 8000
        for i = 1:size(travel_centers,2)-1
            
            % init
            %travel_centers
            xT = travel_centers(:,i+1);
            pRef.setTransition(state(4:6),xT,t_start);

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
    
    
    %%%%%% Graph
    figure()
    hold on
    plot3(X_total(:,1),X_total(:,2),X_total(:,3))
    plot3(X_total(:,4),X_total(:,5),X_total(:,6))
    plot3(centers(1,:),centers(2,:),centers(3,:))
    hold off
    
    %%%%% Phi
    figure()
    hold on
    grid on
    box on
    title('\phi_1 (x) weights')
    plot(T_total, X_total(:,7:10))
    hold off
    
    figure()
    hold on
    grid on
    box on
    title('\phi_2 (x) weights')
    plot(T_total, X_total(:,11:14))
    hold off
    
    %%%%% Gamma
    % g11
    figure()
    hold on
    grid on
    box on
    title('\gamma_{11}(x) weights')
    plot(T_total, X_total(:,15:18))
    hold off
    
    % g21
    figure()
    hold on
    grid on
    box on
    title('\gamma_{21}(x) weights')
    plot(T_total, X_total(:,19:22))
    hold off
    
    % g12
    figure()
    hold on
    grid on
    box on
    title('\gamma_{12}(x) weights')
    plot(T_total, X_total(:,23:26))
    hold off
    
    % g22
    figure()
    hold on
    grid on
    box on
    title('\gamma_{22}(x) weights')
    plot(T_total, X_total(:,27:30))
    hold off
    
    save('SISO_identification.mat');
    
end
function dx = test_plant( t,x,u,controller)
    dx = zeros(3,1);
    
    Phi = [ 1, 2, -2, 1;
            1, 1.5, 2, 2.5] * controller.approximator.Zf(x);
        
    Gamma = [0.4, 0.8, 1.2 1.6;
            -0.1, -0.2 , -0.3, -0.4;
             0.3, 0.6, 0.9, 1.2
             0.5, 1.0, 1.5, 2.0] * controller.approximator.Zg(x);
         
    Gamma = reshape(Gamma,2,2);
        
    f = Gamma\Phi;
     
    % 1st subsystem
    dx(1) = x(2);
    dx([2,3]) = f + Gamma\u;


end
function dx = closed_loop(t,x,ref,controller)

    dx = zeros(size(x,1),1);
    % States
    x_plant = x(1:3);
    x_ref   = x(4:6);
    
    weights = x(7:end);
    controller.approximator.setWeights(weights);
    
    % Calculate
    u_ref = ref.controlInput(t);
    u = controller.controlLaw(t,x_plant,x_ref, u_ref);
    
    % Plant
    dx(1:3) = test_plant(t,x_plant,u,controller);
    
    % Reference Plant
    dx(4:6) = ref.plant(t,x_ref);
    dx(7:end) = controller.approximator.estimatorDerivatives(x_plant,controller.innovationTerms);
end
