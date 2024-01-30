function plotUAV(center,rot,wspan,wblades)
if nargin == 3
    wblades = 20;
end
    pt1 =  center + wspan.*[cos(rot+pi/4) sin(rot+pi/4)];
    pt2 =  center - wspan.*[cos(rot+pi/4) sin(rot+pi/4)];
    pt3 =  center - wspan.*[cos(rot-pi/4) sin(rot-pi/4)];
    pt4 =  center + wspan.*[cos(rot-pi/4) sin(rot-pi/4)];
    
    body = [pt1;pt2;pt3;pt4];
    
    plot(body(1:2,1),body(1:2,2),'k','LineWidth',2.5)
    plot(body(3:4,1),body(3:4,2),'k','LineWidth',2.5)
    scatter(body(:,1),body(:,2),wblades,[0.5,0.5,0.5],'filled','MarkerFaceAlpha',0.6, 'MarkerEdgeColor','k')
end