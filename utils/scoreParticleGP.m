function score = scoreParticleGP(pose,angles,obs,goal,gCenter,gp,options,maxRange)
    [hits, hitCoords, hitsAngles] = rayTraceGP(pose,angles,obs,goal,gCenter,1,maxRange); % 0 nothing, 1 goal, -1 obstacle
    % [hits, hitCoords, hitsAngles] = rayTraceVehicle(pose(1:2),gCenter,angles,obs,goal,1,maxRange); % 0 nothing, 1 goal, -1 obstacle
    if(any(vecnorm(obs-pose, 2,2)<5))
        score = 0;
        return;
    end
    % filter out the part of the target that has already been seen
    seenthreshold = 0.9;
    newPart = goal(gp.mu < seenthreshold, :);
    if isempty(newPart) || isempty(hitCoords) || sum(hits) < 2
        score = 0;
        return;
    end
    
    if size(unique(newPart(:,1)),1) > 1 % look at x direction
        idx = (hitCoords(:,1) > min(newPart(:,1)) & hitCoords(:,1) < max(newPart(:,1))) & logical(hits);
    else    % look at y direction
        idx = (hitCoords(:,2) > min(newPart(:,2)) & hitCoords(:,2) < max(newPart(:,2))) & logical(hits);
    end
    hits = hits(idx, :);
    hitCoords = hitCoords(idx, :);
    hitsAngles = hitsAngles(1, idx);
    if sum(hits) < 2
        score = 0; 
        return;
    end
    
    % update gp for particle
    gp_particle = gp;
    gp_particle = updateGP(gp_particle, hitCoords, hits, options);

    PHI =  2*pi/4;                      % !!!--------- This will be replaced by camera settings/the second set of ray casting
    % PHI = angles(end) - angles(1);
    occupyHandle = @gammaOccupy;
    perspectiveHandle = @gammaPerspective;
    % hit = reshape(hits, r_ang, c_ang);
    whichGoalHits = hitCoords(logical(hits),:);

    %% compute gamma_occupy
    targetAngles = hitsAngles(1,logical(hits));
    phi_min = min(targetAngles);
    phi_max = max(targetAngles);
    
    percentage = abs(phi_max-phi_min)/PHI;
    gamma_occupy = occupyHandle(percentage);

    %% compute gamma_perspective
    % x-y plane for gamma_perspective_horizontal
    if length(unique(whichGoalHits(:,2))) == 1
        % target is aligned with x axis
        [~, i_min] = min(whichGoalHits(:,1));
        [~, i_max] = max(whichGoalHits(:,1));
    else
        % target is aligned with y axis
        [~, i_min] = min(whichGoalHits(:,2));
        [~, i_max] = max(whichGoalHits(:,2));
    end
    lT_h = norm(whichGoalHits(i_max,:)-whichGoalHits(i_min,:));
    if (lT_h ~= 0)
	    l1_h = norm(whichGoalHits(i_min, 1:2) - pose(1:2));
 	    l2_h = norm(whichGoalHits(i_max, 1:2) - pose(1:2));
	    gamma_perspective = perspectiveHandle(abs(l1_h-l2_h)/lT_h);
    else
	    gamma_perspective = 0;
    end

    %% compute info gain
    % curr_info =  (gp_particle.mu > 0.85);
    % pref_info =  (gp.mu > 0.85);
    % new_info = curr_info - pref_info;
    % new_info(new_info < 0) = 0;
    % infoGain = sum(new_info)/size(gp.mu, 1);
    infoGain = sum(gp_particle.mu)/size(gp.mu, 1);
    
    %% compute score
    % score = infoGain;
    % score = gamma_perspective * infoGain;
    % score = gamma_occupy * infoGain;
    score = gamma_occupy * gamma_perspective * infoGain;
end