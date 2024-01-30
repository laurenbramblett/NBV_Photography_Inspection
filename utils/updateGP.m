function gp = updateGP(gp,hitCoords,goalHits,options)
    gp.X_train_data = [gp.X_train_data; hitCoords];
    %Add number of hits to the training data
    gp.y_train_data = [gp.y_train_data; goalHits];
    %Find best theta fit for data
    if gp.update 
        gp.theta = fminunc(@(theta) nll_stable(gp,theta),gp.theta,options);
        gp.update = 0;
    end
    %Update hit likelihood
    [gp.mu, gp.cov] = posterior(gp.X_train,gp);
end