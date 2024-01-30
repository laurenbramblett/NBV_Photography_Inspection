%plotInlineRay
GP{1}.mu = zeros(size(goalOriginal{1},1),1);
% [hits, hitCoords, ~] = rayTraceGP(vehicle(1:2)',angles,knownObs,goal{goalIndex},gCenter{goalIndex},gridRes,maxRange); % 0 nothing, 1 goal, -1 obstacle
if step == 2
    allPositions = [];
end
hold on;
imagesc(x,y,reshape(M0,mapSize)); colormap(flipud(bone));
axis xy; axis equal;

% plot particles
scatter(particle(:,1),particle(:,2),20,'MarkerFaceColor','b','MarkerEdgeColor','b','MarkerFaceAlpha',0.5)
% plot optimal inspecting point
scatter(gBest_pos(1),gBest_pos(2),40,'MarkerFaceColor','r','MarkerEdgeColor','k','MarkerFaceAlpha',0.8)
% plot vehicle
allPositions = [allPositions; vehicle(1:2)'];
plot(allPositions(:,1),allPositions(:,2),'r','LineWidth',1)
plotUAV([vehicle(1) vehicle(2)], vehicle(3),1);

if size(pictureTakenCoords,1)>0
    scatter(pictureTakenCoords(:,1),pictureTakenCoords(:,2),40,'y','filled')
end

ax1 = gca;

if lastFrame
    goalIndex = goalIndex - 1;
end

% plot current GP
ax2 = axes('position', get(ax1, 'position')); hold on;
x2d = [min(goal{goalIndex}(:,1)), max(goal{goalIndex}(:,1))] + plotoffset{goalIndex}(1, 1);
y2d = [min(goal{goalIndex}(:,2)), max(goal{goalIndex}(:,2))] + plotoffset{goalIndex}(1, 2);
rescaleGP  =GP{goalIndex}.mu>0.15;
if ~isempty(x2d)
    rescaleGP((rescaleGP > 1), 1) = 1;
    imagesc(x2d, y2d, [ones(size(GP{goalIndex}.mu)), ...
                       zeros(size(GP{goalIndex}.mu)),...
                       repmat(rescaleGP,1,1)]...
                       );
end

colormap(ax2, 'jet');
set(ax2,'color','none','visible','off','XTick',[],'YTick',[]);
set(ax2,'ydir','normal');
ax2.UserData = linkprop([ax1,ax2],...
    {'Position','InnerPosition','DataAspectRatio','xtick','ytick', ...
    'ydir','xdir','xlim','ylim'});
% axis(ax2, 'equal');
% axis(ax2, 'xy');


% plot patches to cover the GP padding
ax3 = axes('position', get(ax1, 'position')); hold on;
x3d = [min(goal{goalIndex}(:,1)), max(goal{goalIndex}(:,1))] + plotoffset{goalIndex}(2, 1);
y3d = [min(goal{goalIndex}(:,2)), max(goal{goalIndex}(:,2))] + plotoffset{goalIndex}(2, 2);
if length(unique(goal{goalIndex}(:,1))) > 1
    imagesc(x3d, y3d, [zeros(size(GP{goalIndex}.mu)), ones(size(GP{goalIndex}.mu))]');
else
    if ~isempty(x3d)
        imagesc(x3d, y3d, [zeros(size(GP{goalIndex}.mu)), ones(size(GP{goalIndex}.mu))]);
    end
end
colormap(ax3, [0, 0, 0]);
set(ax3,'color','none','visible','off','XTick',[],'YTick',[]);
set(ax3,'ydir','normal');
ax3.UserData = linkprop([ax1,ax3],...
    {'Position','InnerPosition','DataAspectRatio','xtick','ytick', ...
    'ydir','xdir','xlim','ylim'});

xlim([x(1) x(end)]);
ylim([y(1) y(end)]);

xlim([x(1) x(end)]);
ylim([y(1) y(end)]);

if lastFrame
    goalIndex = goalIndex + 1;
end


