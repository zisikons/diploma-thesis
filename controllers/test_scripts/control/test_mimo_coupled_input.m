function [] = test_mimo_coupled_input()
    %% Include utilities to path
    g = genpath('~/diploma-thesis/utilities/');
    addpath(g)
    
    g = genpath('~/diploma-thesis/controllers/');
    addpath(g)
    
    close all

    %% System Initiallization
    % PPC Utilities
    per = PerfomanceFunction(2,2,0.01);
    trans = PPCTransformation();

    % Controller
    system_form = [2,1]; 
    PPC_controller = SimplePPController(system_form,per,trans);
    
    % Reference system
    DT = 1;
    pRef = PolynomialReference(system_form,DT);
    
    % Centers to visit
    centers = [0 0 0;
               0 1 1;
               0 1 -2;
               1 1 4];
           
    centers = centers';
    t_start = 0;
    T_sim = [0,DT];
    
    T_total = [];
    X_total = [];
    
    state = repmat(centers(:,1),2,1);
    
    %% Control loop
    for i = 1:size(centers,2)-1
        % init
        x0 = centers(:,i);
        xT = centers(:,i+1);
        pRef.setTransition(x0,xT);  % better use state instead of x0
        
        % parameterize loop
        sim_loop = @(t,x) closed_loop(t,x,pRef,PPC_controller,t_start);
        
        % Simulate
        [T,X] = ode15s(sim_loop,T_sim,state);
        t_start = T(end);
        T_sim = T_sim + DT;
        state = X(end,:)';
        
        T_total = [T_total;T];
        X_total = [X_total;X];
        
    end
        

    [U,KSI,E] =  PPC_controller.controlLaw(T_total',...
                                           X_total(:,[1,2,3])',...
                                           X_total(:,[4,5,6])');
                                       
    V = PPC_controller.leftoverErrors(T_total',...
                                  X_total(:,[1,2,3])',...
                                  X_total(:,[4,5,6])');

    figure()
    hold on
    plot3(X_total(:,1),X_total(:,2),X_total(:,3))
    plot3(X_total(:,4),X_total(:,5),X_total(:,6))
    plot3(centers(1,:),centers(2,:),centers(3,:),'rx')
    hold off

    figure()
    hold on
    title('errors')
    plot(T_total,E)
    hold off
    
    figure()
    hold on
    title('ksi')
    plot(T_total,KSI)
    hold off
    

    figure()
    hold on
    title('control input')
    plot(T_total,U)
    hold off
    
    figure()
    hold on
    title('leftovers')
    plot(T_total,V)
    hold off
  
end
function dx = test_plant(t,x,u)
    dx = zeros(3,1);
    
    % 1st subsystem
    dx(1) = x(2);
    dx(2) = -x(3)^2 + sin(x(1))*x(2) + 3*(1+x(2)^2)*u(1)  + 2*u(2);
    
    % 2nd subsystem
    dx(3) = x(3) + 0.1*x(2) + 2*sin(t)*u(1)  + 5*u(2);
end
function dx = closed_loop(t,x,ref,controller,t_start)
    dx = zeros(6,1);
    % States
    x_plant = x(1:3);
    x_ref   = x(4:6);
    
    % Calculate 
    u = controller.controlLaw(t,x_plant,x_ref);
    
    % Plant
    dx(1:3) = test_plant(t,x_plant,u);
    
    % Reference Plant
    dx(4:6) = ref.plant(t-t_start,x_ref);
end