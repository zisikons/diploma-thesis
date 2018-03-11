function [u,ksi,e] = controlLaw(obj, t, x, x_ref, u_ref)
    % Innovation term
    v = -u_ref + obj.leftoverErrors(t, x, x_ref);

    % It works
    [u,ksi,e] = controlLaw@SimplePPController(obj,t,x,x_ref);

    u = u - obj.approximator.Phi(x) ...
          - obj.approximator.Gamma(x) * (v - ksi*obj.performance.rhoDot(t));

    % Update Innovation Terms
    obj.innovationTerms.Phi   = ksi./obj.performance.rho(t);
    obj.innovationTerms.Gamma = ksi./obj.performance.rho(t) * (v - ksi*obj.performance.rhoDot(t))' ;
end