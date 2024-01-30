function [mu_s,cov_s] = posterior(X_s, gp)
%%%
%     Computes the suffifient statistics of the posterior distribution 
%     from m training data X_train and Y_train and n new inputs X_s.
%     
%     Args:
%         X_s: New input locations (n x d).
%         gp.X_train: Training locations (m x d).
%         gp.y_train: Training targets (m x 1).
%         gp.noise: Kernel length parameter.
%         gp.theta(1): Kernel vertical variation parameter.
%         gp.theta(2): Noise parameter.
%     
%     Returns:
%         Posterior mean vector (n x d) and covariance matrix (n x n).
%%%
    sigma_f = gp.theta(1); sigma_y = gp.theta(2); l = gp.noise;
    X_train = gp.X_train_data; Y_train = gp.y_train_data;
    K = kernel(X_train, X_train, l, sigma_f) + sigma_y^2 * eye(length(X_train));
    K_s = kernel(X_train, X_s, l, sigma_f);
    K_ss = kernel(X_s, X_s, l, sigma_f) + sigma_y^2 * eye(length(X_s));
    % K_inv = inv(K);
    
%     Equation (7)
    mu_s = K_s'*(K\Y_train);

%     Equation (8)
    cov_s = K_ss - K_s'*(K\K_s);
    
end