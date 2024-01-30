function res = kernel(X1,X2,l,sigma_f)
    sqdist = sum(X1.*X1, 2) + sum(X2.*X2, 2)' - 2 *(X1*X2');
    res = sigma_f^2 * exp(-0.5 / l^2 * sqdist);
end