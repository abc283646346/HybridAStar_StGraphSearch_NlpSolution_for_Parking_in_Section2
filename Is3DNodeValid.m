function is_collision_free = Is3DNodeValid(x, y, theta)
is_collision_free = 1;
global vehicle_geometrics_
global planning_scale_
global hybrid_astar_
global costmap_

xr = x + vehicle_geometrics_.r2x * cos(theta);
yr = y + vehicle_geometrics_.r2x * sin(theta);
xf = x + vehicle_geometrics_.f2x * cos(theta);
yf = y + vehicle_geometrics_.f2x * sin(theta);
xx = [xr, xf];
yy = [yr, yf];
if (length(find(xx > planning_scale_.xmax)) + length(find(xx < planning_scale_.xmin)) + length(find(yy > planning_scale_.ymax)) + length(find(yy < planning_scale_.xmin)) > 0)
    is_collision_free = 0;
    return;
end

indxf = round((xf - planning_scale_.xmin) /  hybrid_astar_.resolution_x + 1);
indyf = round((yf - planning_scale_.ymin) /  hybrid_astar_.resolution_y + 1);
indxr = round((xr - planning_scale_.xmin) /  hybrid_astar_.resolution_x + 1);
indyr = round((yr - planning_scale_.ymin) /  hybrid_astar_.resolution_y + 1);
for ii = 1 : length(xr)
    if ((costmap_(indxf(ii), indyf(ii)) == 1)||(costmap_(indxr(ii), indyr(ii)) == 1))
        is_collision_free = 0;
        return;
    end
end
end