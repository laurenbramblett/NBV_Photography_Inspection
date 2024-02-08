clc; clear all; close all;
profile clear;
profile off;
addpath('utils');
set(0,'DefaultFigureWindowStyle','docked');

% compile hybrid A* to cpp file for speeding up 
numCells = 100; % Resolution of the map
if ~isfolder('codegen')
    mapData = zeros(numCells); startPose = zeros(1,3); goalPose = zeros(1,3); infl = 5;
    codegen -config:mex codegenPathPlanner -args {mapData,startPose,goalPose,infl}
else
    addpath(genpath('./codegen'))
end

% Initialize environment
gridRes = 1; 
dt = 0.1;
step = 1;
x = 1:gridRes:100;
y = 1:gridRes:100;
[X,Y] = meshgrid(x,y);
mapSize = size(X);
gridPts = [X(:),Y(:)];


% This is the part I define the envirnment
%-------------------------------------------------------------------------------------------------
M0 = ones(size(X))*0.5;
% boundaries
M0(1:100,1:2) = 1;
M0(1:100,99:100)=1;
M0([1,2,99,100],1:100) = 1;
% |-------3-------|
% |               |
% 4               2
% |               |
% |-------1-------|
% Example observation of one side of a house
house = [35, 35; 40, 60]; % bottom left; top right. Obs is one unit inside
goal{1} = [repelem(house(2,1)+1, length(house(1,2):house(2,2)), 1), (house(1,2):house(2,2))'];
goalOriginal = goal;
% Let all the house be unseen
unSeenIdx = true(size(goal,1));
% Define the house center
gCenter{1} = mean(goal{1},1);
[obs_house_X, obs_house_Y] = meshgrid(house(1,1):house(2,1), house(1,2):house(2,2));
obs_house = [obs_house_X(:), obs_house_Y(:)];
obs_houseIdx = any(all(bsxfun(@eq,reshape(obs_house',1,2,[]),gridPts),2),3);
M0(obs_houseIdx) = 1;

% add one unknown obstacle in front of the house
% surface 1: T horizontal bar
obs1 = [(40:53)', repelem(52, length(40:53), 1)];
% surface 1: T vertical bar
obs2 = [repelem(53, length(43:55), 1), (43:55)'];

% all obstacles (includeing the unkown ones)
obs = [obs_house; obs1; obs2];

% if sim want to start with obstacles remain unknown
knownObs = [obs_house]; % obstacles that are known as a priori
% if sim want to start with all obstacles known
% knownObs = obs;
obs1_Idx = any(all(bsxfun(@eq,reshape(obs1',1,2,[]),gridPts),2),3);
obs2_Idx = any(all(bsxfun(@eq,reshape(obs2',1,2,[]),gridPts),2),3);

M0_GT = M0; % ground truth of the map
M0_GT(obs1_Idx) = 1;
M0_GT(obs2_Idx) = 1;

imagesc(M0_GT); colormap(flipud(bone)); axis xy; %See what final explored view would look like
%-------------------------------------------------------------------------------------------------

% thresholds & flags
goalIndex = 1; % current index for goal, GP
finishThresh = 0.1; % threshold for switching to the next goal available
picThresh = 0.5; % distance from the optimal position for allowing taking pic
finished = false;
lastFrame = false;
coolDownTimer = 5;
coolDownCnt = 0;
takePhoto = false;
gBest_changed = false;

% Initialize vehicle
vehicle = [30; 10; 0];
max_vel = 12; 
max_yaw = pi/8;
% Initialize senor
angleRes = pi/32;
angles = -pi/4:angleRes:pi/4; 
maxRange = 25;
robot.angles = angles;
robot.maxRange = maxRange;
robot.pose = vehicle;

% PSO Init
x_lims = [x(1) x(end)];
y_lims = [y(1) y(end)];
numParticles = 40; % 40 worked
particle = [(x_lims(end)-x_lims(1))*rand(numParticles,1)+x_lims(1) ...
            (y_lims(end)-y_lims(1))*rand(numParticles,1)+y_lims(1)];
particleVel = randn(numParticles,2);
value = zeros(numParticles,1);%+100;
pBest = particle;
pBest_obj = value;
[gBest_obj,gIdx] = max(pBest_obj);
gBest_pos = pBest(gIdx,:);
gBest_val = pBest_obj(gIdx); 
numIters = 5;

% PSO Params
c1 = 0.1;
c2 = 0.1;
w = 0.8;
distThresh = 1; %Resampling Threshold
% n_steps = 400;


% Path Planning parameters
goalPrev = [0,0,0]; nextGoal = ones(1,3)*1000;
change_goalThreshold = 5;
change_pathThreshold = 3;
infl = 3;

% GPOM INIT
gp1 = struct('theta',[],'noise',0,'X_train',[],'y_train',[],'mu',0,'cov',0,'update',0);
gp1.theta    = [1,1]; 
gp1.noise    = 1;
gp1.X_train  = goal{1};
gp1.y_train  = zeros(size(gp1.X_train,1),1);
gp1.X_train_data = []; 
gp1.y_train_data = [];
gp1.mu       = zeros(size(gp1.X_train,1),1); 
gp1.cov      = zeros(size(gp1.X_train,1));

% Set solver options for GP
options     = optimoptions('fminunc', 'Display', 'none', 'MaxIter', 1000,...
                           'HessianApproximation',"lbfgs");

GP = {gp1};%, gp2, gp3, gp4};
clear gp1 gp2 gp3 gp4;

GP_plotoffset{1} = [-2, 0; -2, -0];


% Initialize path plan
pthObj = [];
timeStamp = datestr(now, 'HH_MM_SS');

% keep appending
saveData.GP = GP;
saveData.vehicle = vehicle;
saveData.knownObs = knownObs;
saveData.gCenter = gCenter;
saveData.takePhoto = takePhoto;
saveData.goalIndex = goalIndex;
saveData.particle = particle;
saveData.gBest_pos = gBest_pos;
saveData.pthObj = pthObj;
saveData.lastFrame = lastFrame;
saveData.gridPts = gridPts;
saveData.endPts = [];
saveData.M0 = M0;

% save once
megaData.angles = angles;
megaData.obs = obs;
megaData.goal = goal;
megaData.GP_plotoffset = GP_plotoffset;
megaData.options = options;
megaData.data{step} = saveData;
megaData.x = x;
megaData.y = y;
megaData.mapSize = mapSize;
megaData.maxRange = maxRange;
megaData.M0_GT = M0_GT;
storeVehicle = vehicle;
pictureTakenCoords = [];


%% Start 
while ~finished %|| step < 1200
    tic;
    % vehicle(3) = atan2(gCenter{goalIndex}(2)-vehicle(2),gCenter{goalIndex}(1)-vehicle(1));
    % ============================================== update map using lidar
    robot.pose = vehicle;
    [z, ~, ~, endPts] = lidarSim_v2(robot,obs,mapSize); % obs index of occgrid
    
    obstacles_beforeUpdate = (M0 > .85);
    M0 = updateOccupancyMap(z,M0,mapSize);
    obstacles_afterUpdate = (M0 > .85);
    if ~isequal(obstacles_beforeUpdate, obstacles_afterUpdate)
        pBest_obj(:) = 0;
        particle = [(x_lims(end)-x_lims(1))*rand(numParticles,1)+x_lims(1) ...
            (y_lims(end)-y_lims(1))*rand(numParticles,1)+y_lims(1)];
    end
    knownObs = gridPts(M0 > .85,:);
    % rescore the gBest_val
    gBest_val = scoreParticleGP(gBest_pos,angles,knownObs,goal{goalIndex},gCenter{goalIndex},GP{goalIndex},options, maxRange);
    % ================================================================= PSO
    for i = 1:numIters 
        for p = 1:numParticles
            distCalc = sqrt(sum((particle(p,:)-gBest_pos).^2));
            if distCalc<distThresh && ~isequal(particle(p,:),gBest_pos)
                particle(p,:) = [(x_lims(end)-x_lims(1))*rand()+x_lims(1) ...
                                 (y_lims(end)-y_lims(1))*rand()+y_lims(1)];
            end
            pVehicle = particle(p,:);
            pAng = atan2(mean(goal{goalIndex}(2,:))-pVehicle(2), mean(goal{goalIndex}(1,:))-pVehicle(1));
            value(p) = scoreParticleGP(pVehicle(1:2),angles,knownObs,...
                goal{goalIndex},gCenter{goalIndex},GP{goalIndex},options,maxRange);
            % Change sign if maximizing
            if value(p)>pBest_obj(p)
                pBest_obj(p) = value(p);
                pBest(p,:) = particle(p,:);
            end         
        end
        % Check best particles for global min -> change to max if maximizing
        [gBest_obj,gIdx] = max(pBest_obj);
        % Decide if the new best is worth going
        val_threshold = minVal4Switch(pBest(gIdx,:), gBest_pos, vehicle(1:2)', pBest_obj(gIdx), gBest_val);
        if pBest_obj(gIdx) > gBest_val
            if pBest_obj(gIdx) - gBest_val >= val_threshold
                gBest_pos = pBest(gIdx,:);   % the result of the particle swarm
                gBest_changed = true;
            end
        end
        r = rand(numParticles,2);
        particleVel = w*particleVel + c1*r(:,1).*(pBest - particle) + c2*r(:,2).*(gBest_pos-particle);
        particle = particle + particleVel;
        particle(particle(:,1)<x_lims(1),1) = x_lims(1);
        particle(particle(:,1)>x_lims(2),1) = x_lims(2);
        particle(particle(:,2)<y_lims(1),2) = y_lims(1);
        particle(particle(:,2)>y_lims(2),2) = y_lims(2);
    end
            
    % focus on the way to the next position is far from the viewing position
    bestDist = pdist2(vehicle(1:2)',gBest_pos);
    
    if pdist2(gBest_pos(1:2),goalPrev(1:2))> change_goalThreshold || size(pthObj,1)<=1 ||gBest_changed
        gBestAng = atan2(mean(goal{goalIndex}(2,:))-gBest_pos(2), mean(goal{goalIndex}(1,:))-gBest_pos(1));
        try
            pthObj = mapHybridAStarGrid(vehicle',[gBest_pos,gBestAng],M0,infl);
            pthObj = pthObj(2:end,:);
            nextGoal = pthObj(1,:);
            goalPrev = gBest_pos;
        catch
            if size(pthObj,1)>1 && pdist2(vehicle(1:2)',nextGoal(1:2)) < change_pathThreshold
                pthObj = pthObj(2:end,:);
                nextGoal = pthObj(1,:);
            elseif bestDist < change_goalThreshold
                nextGoal = gBest_pos;
            end
        end
    elseif pdist2(vehicle(1:2)',nextGoal(1:2)) < change_pathThreshold
        pthObj = pthObj(2:end,:);
        nextGoal = pthObj(1,:);
    elseif bestDist < change_goalThreshold
        nextGoal = gBest_pos;
    end
    gBest_changed = false;
    
    % ======================================================== move vehicle
    if bestDist < 5
        nextGoal = gBest_pos;
    end

    goalSpring = nextGoal(1:2)-vehicle(1:2)';
    if norm(goalSpring)>max_vel
        vel = goalSpring/norm(goalSpring)*max_vel;
    else
        vel = goalSpring;
    end
    vehicle(1) = vehicle(1) + vel(1)*dt;
    vehicle(2) = vehicle(2) + vel(2)*dt;
    vehicle(3) = atan2(gCenter{goalIndex}(2)-vehicle(2), gCenter{goalIndex}(1)-vehicle(1));
    
    % ================================= take pic + update gp + draw heatmap
    if bestDist<picThresh && coolDownCnt == 0
        % ==================================================== draw heatmap
        % totalScore = plotRayTraceFieldGP(gridPts,angles,obs,goal{goalIndex},gCenter{goalIndex},GP{goalIndex},options);
        % [M,I] = max(totalScore);
        % bestPosition = gridPts(I,:);
        % title(sprintf('Best position is %.2f, %.2f of %.3f, vehicle is at %.2f, %.2f\n', bestPosition(1), bestPosition(2), M, vehicle(1), vehicle(2)));
        % figName = sprintf('./SimResult/heatmap_%s_%s.fig', timeStamp, num2str(step));
        takePhoto = true;
        pictureTakenCoords = [pictureTakenCoords; vehicle(1:2)'];
        % ======================================================= Update GP
        [hit,hitCoords,~] = rayTraceGP(vehicle(1:2)',angles,knownObs,goal{goalIndex},gCenter{goalIndex},gridRes,maxRange);
        % Check which segments have been hit
        GP{goalIndex}.update = 1;
        GP{goalIndex} = updateGP(GP{goalIndex},hitCoords,hit,options);
        % update goalCenter
        unSeenIndex = GP{goalIndex}.mu < 0.2;
        gCenter{goalIndex} =  mean(goalOriginal{goalIndex}(unSeenIndex, :), 1);
        
        % ================================================== draw GP result
        % if norm(vehicle(1:2)'-bestPosition) > picThresh
        %     fprintf('Vehicle(%.3f, %.3f) is not at the best position (%.3f, %.3f)\n', vehicle(1), vehicle(2), bestPosition(1), bestPosition(2));
        % end

        % savefig(figName);
        % pngName = sprintf('./SimResult/heatmap_%s_%s.png', timeStamp,  num2str(step));
        % saveas(gcf,pngName);
        % close;
        
        % ============================================= switch to next goal
        % progress = sum(GP{goalIndex}.mu)/size(goalOriginal{goalIndex},1);
        coolDownCnt = coolDownTimer;
        % if (1-progress) < finishThresh || all(GP{goalIndex}.mu > 0.15)
        if all(GP{goalIndex}.mu > 0.15)
            goalIndex = goalIndex + 1;
            lastFrame = true;
            if goalIndex == 2
                goalIndex = 1;
                finished = true;
                lastFrame = false;
            end
        end
        progress = sum(GP{goalIndex}.mu > 0.15)/size(goalOriginal{goalIndex},1);
        fprintf('Working on Target %d, Progress: %.2f%%\n', goalIndex, progress*100);
        
        % ================= randomize particles after taking the picture
        pBest_obj(:) = 0;
        particle = [(x_lims(end)-x_lims(1))*rand(numParticles,1)+x_lims(1) ...
            (y_lims(end)-y_lims(1))*rand(numParticles,1)+y_lims(1)];
    end
    toc

    step = step + 1;
    cla;
    plotInlineRay_approachExample
    if coolDownCnt == 0
        if lastFrame
            lastFrame = false;
        end
    else
        coolDownCnt = coolDownCnt - 1;
    end
    
    drawnow;
    pause(0.01);
    saveData.GP = GP;
    saveData.vehicle = vehicle;
    saveData.knownObs = knownObs;
    saveData.gCenter = gCenter;
    saveData.takePhoto = takePhoto;
    saveData.goalIndex = goalIndex;
    saveData.particle = particle;
    saveData.gBest_pos = gBest_pos;
    saveData.pthObj = pthObj;
    saveData.lastFrame = lastFrame;
    saveData.gridPts = gridPts;
    saveData.endPts = endPts;
    saveData.M0 = M0;

    megaData.data{step} = saveData;

    if step == 2
        frame = getframe(gcf);
    else
        frame(end+1) = getframe(gcf);
    end
    delete(ax2);
    delete(ax3);
    takePhoto = false;
end

for i = 1:coolDownTimer
    frame(end+1) = frame(end);
end

save(sprintf([timeStamp, 'approach','.mat']));

videoName = sprintf([timeStamp,'approach', '_1']);
video = VideoWriter(videoName, 'MPEG-4');
video.FrameRate = 10;
open(video);
for i = 1:1:length(frame)
%     writeVideo(video, frame(i).cdata(:,1:1150,:));
    writeVideo(video, frame(i));
end
close(video);




