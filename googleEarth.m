uif = uifigure;
g = geoglobe(uif);
hold(g, "on");

% figpos = [1000 500 800 400];
% uif = uifigure("Position",figpos);
% ug = uigridlayout(uif,[1,2]);
% p1 = uipanel(ug);
% p2 = uipanel(ug);
% gx = geoaxes(p1,"Basemap","satellite"); 
% hold(gx,"on");
% gg = geoglobe(p2); 
% hold(gg,"on");
% gx.InnerPosition = gx.OuterPosition;
% gg.Position = [0 0 1 1];


%%
lat = [39.95 39.95]
lon = [-107 -107 ]
hTerrain = [1000 1000];

dy=0.3
dx= 0.02
%%
for j = 1:10
    if  rem(j,4) == 1
        lon(1) = lon(2);
        lon(2) = lon(2);
        lat(1) = lat(2);
        lat(2) = lat(2)+dy;
        
    elseif rem(j,4) == 2
        lon(1) = lon(1);
        lon(2) = lon(2)+dx;
        lat(1) = lat(2);
        lat(2) = lat(2);
        
    elseif rem(j,4) == 3
        lon(1) = lon(2);
        lon(2) = lon(2);
        lat(1) = lat(2);
        lat(2) = lat(2)-dy;
        
    elseif rem(j,4) == 0
        lon(1) = lon(2);
        lon(2) = lon(2)+dx;
        lat(1) = lat(2);
        lat(2) = lat(2);
        
    end
    geoplot3(g,lat,lon,hTerrain,'y','HeightReference','terrain', 'LineWidth',3)
    drawnow
    
    
end