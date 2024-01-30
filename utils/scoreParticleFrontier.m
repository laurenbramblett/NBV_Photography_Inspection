function score = scoreParticleFrontier(pose,angles,obs,goal,gCenter,maxRange,vehicle)
    [hits, ~, ~, whichHits] = rayTraceGP_frontier(pose,angles,obs,goal,gCenter,1,maxRange); % 0 nothing, 1 goal, -1 obstacle
    whichHits = unique(whichHits,"rows");
    numHits = size(whichHits,1);
    score = -(numHits)/size(angles,2) + 1/40*norm(pose(1:2)-vehicle(1:2)');
    close2Obs = any(vecnorm(pose-obs,2,2)<3);


    if sum(hits)<2 || close2Obs
        score = 100;
        return
    end
    fprintf("Number of hits: %f\n", numHits)
    fprintf("Distance: %f\n",norm(pose(1:2)-vehicle(1:2)'))
    fprintf("Score: %f\n", score)
end