function [newHits, newCoords, newangles,goalHitCoords] = rayTraceGP_frontier(pose, angles, obs, goal, cGoal, rad, maxRange)
    pt = pose;
    theta_d = atan2(cGoal(2) - pt(2), cGoal(1) - pt(1));
    numAngles = length(angles);
    hitCoords = zeros(numAngles, 2); 
    goalHit = zeros(numAngles, 1);

    whichGoalsCoords = zeros(numAngles,2);

    ang = mod(theta_d + angles + pi, 2 * pi) - pi;
    adj_angles = [cos(ang); sin(ang)]';
    pt2 = pt + adj_angles;
    d = pt2-pt;
    rayTerms = pt + maxRange * adj_angles;
    % d = rayTerms - pt;

    % Precompute norms
    d_norm = sqrt(sum(d.^2, 2));

    % Only check obs within range
    obsDist = sqrt(sum((pt(1:2)-obs).^2,2));
    obsSubIdx = obsDist<=maxRange;
    obsSub = obs(obsSubIdx,:);
    % obsSubDist = obsDist(obsSubIdx);

    %Only check goals within range
    goalDist = sqrt(sum((pt(1:2)-goal).^2,2));
    goalSubIdx = goalDist<=maxRange;
    % goalDistSub = goalDist(goalSubIdx);
    goalSub = goal(goalSubIdx,:);
    for a = 1:numAngles
        % Obs
        if isempty(obsSub)
            obsInter = 0;
        else
            [obsInter, coordTermObs,~] = checkIntersections(obsSub, pt, d(a, :), rad, d_norm(a));
        end

        % Goal
        if isempty(goalSub)
            goalInter = 0;
        else
            [goalInter, coordTermGoal,roundTerm] = checkIntersections(goalSub, pt, d(a, :), rad, d_norm(a));
        end

        % Determine hit coordinates and goal hit status
        if goalInter
            if obsInter && (vecnorm(coordTermGoal - pt) >= vecnorm(coordTermObs - pt))
                hitCoords(a, :) = coordTermObs;
            else
                hitCoords(a, :) = coordTermGoal;
                goalHit(a) = 1;
                whichGoalsCoords(a,:) = roundTerm;
            end
        elseif obsInter
            hitCoords(a, :) = coordTermObs;
        else
            hitCoords(a, :) = rayTerms(a, :);
        end
    end

    % Filter hits based on distance
    include = filterHits(hitCoords);
    newHits = goalHit(include); 
    newCoords = hitCoords(include, :);
    newangles = angles(include);
    goalHitCoords = whichGoalsCoords(logical(goalHit),:);
end

function [intersects, coordTerm, roundTerm] = checkIntersections(targets, pt, d, rad, d_norm)
    intersects = false;
    coordTerm = [0, 0];
    roundTerm = [0,0];
    v = targets - pt;
    dv = v * d'; 
    whichTargets = dv > 0;
    if ~whichTargets
        return
    end
    targets = targets(whichTargets, :);
    dv = dv(whichTargets);
    opt = dv ./ d_norm * d;
    optPrime = pt + opt;
    dist2Pt = sqrt(sum((opt.^2), 2));
    dist2Inter = sqrt(sum((optPrime - targets).^2, 2));
    whichIntersect = find(dist2Inter <= rad);
    if ~isempty(whichIntersect)
        [~, idx] = min(dist2Pt(whichIntersect));
        intersects = true;
        coordTerm = optPrime(whichIntersect(idx), :);
        roundTerm = targets(whichIntersect(idx),:);
    end
end

function include = filterHits(hitCoords)
    numAngles = size(hitCoords, 1);
    include = false(numAngles, 1);
    include(1) = true;
    for a = 2:numAngles
        boolClose = all(vecnorm(hitCoords(include, :) - hitCoords(a, :), 2, 2) > 0.8);
        if boolClose
            include(a) = true;
        end
    end
end
