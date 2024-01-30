function pthObj = mapHybridAStarGrid(x0,target,M0,infl)

M = double(M0>0.85);
M = flipud(M);
map = M;
start = [round(x0(1:2),0),x0(3)];
goal = [round(target(1:2),0),target(3)];
rng(100, 'twister') % repeatable result
% pthObj = codegenPathPlanner_mex(map,start,goal,infl);
pthObj = codegenPathPlanner_mex(map,start,goal,infl);
%pthObj = flip(pthObj,1);
end
% show(planner)
% hold on
% plot(start(2),start(1),'g*',goal(2),goal(1),'r*')
% hold off