function res = nll_stable(gp,theta)
%     # Numerically more stable implementation of Eq. (11) as described
%     # in http://www.gaussianprocess.org/gpml/chapters/RW2.pdf, Section
%     # 2.2, Algorithm 2.1.

    % RBF Kernel to estimate covariance of a prior dist over a target
    % function
    X_train = gp.X_train; Y_train = gp.y_train; noise = gp.noise; 

    K = kernel(X_train, X_train, theta(1), theta(2)) + ...
        noise^2 * eye(length(X_train));

    try
        L = chol(K)';
        opts1.LT = true; opts2.UT = true;
        S1 = linsolve(L, Y_train, opts1);
        S2 = linsolve(L', S1, opts2);
        
        res = sum(log(diag(L))) + ...
               0.5 * dot(Y_train,S2) + ...
               0.5 * length(X_train) * log(2*pi);
    catch
        res = 1e7;
    end
end