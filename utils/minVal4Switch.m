function val_threshold = minVal4Switch(pBest, gBest, vehicle, pv, vBest)
% pBest, gBest, vehicle:        [x; y]  position coordinate
% v, vBest:                 double value should in range of [0,1]
    standardThreshold = 0.12;
    controlDist = 10; % was 7
    % if switch global best make significant increase in info gain, switch
    % anyways
    if (pv-vBest) > standardThreshold
        val_threshold = 0;
        return;
    end
    dist = norm(pBest - gBest);
    % if the next global best is really close to the current global best
    if dist < 5
        val_threshold = 0;
        return;
    end

    dist = norm(vehicle - pBest);
    val_threshold = standardThreshold/(1 + exp(dist-controlDist));

end
