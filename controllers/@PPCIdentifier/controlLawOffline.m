function [u, ksi ,e] = controlLawOffline(obj, t, x, x_ref, u_ref)
%%controlLawOffline Offline closed loop
%
%  There are several reasons for whom there must be a seperate
%  implementation for the offline and online calculations of the
%  identifier's control law. The main one is that there is no way to
%  generate the signal u_ref(T) for the hole time vector because of the
%  internal parameters of ReferenceSystem which may change during the
%  simulation. 
%
%  The alternative is to store the input referece u_ref during the
%  simulation, and use it afterwards to calculate the rest cloded loop
%  signals, which is exactly why this function is used.
%
%  Use this function by providing the matrices T and X by ODE, as well as  
%  the U_ref matrix calculated during the loop.
%  
%  
%  Note: Method not implemented yet, because matlab won't allow to call 
%        vectorized controlLaw@SimplePPController(obj,t,x,x_ref) because
%        of the different name of base and derived methods.
%
%  Solution: Wrapper in base class with name = controlLawOffline( ... )
%            ( MATLAB ffs ... )
%

    % The use of leftoverErrors is possible since it's vectorized
    v = -u_ref + obj.leftoverErrors(t, x, x_ref);
    
    % Vectorized as well however matlab denies ...
    [u,ksi,e] = controlLaw@SimplePPController(obj,t,x,x_ref);
    
    % This one is harder, and this seems to be wrong.
%     u = u - obj.approximator.Phi(x) ...
%           - obj.approximator.Gamma(x) * (v - ksi*obj.performance.rhoDot(t));
    
end