function idx = Convert3DConfigToIndex(config)
global hybrid_astar_ planning_scale_
ind1 = ceil((config(1) - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((config(2) - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;
ind3 = ceil((RegulateAngle(config(3))) / hybrid_astar_.resolution_theta) + 1;
idx = [ind1, ind2, ind3];
end

function angle  = RegulateAngle(angle)
while (angle > 2 * pi + 0.000001)
    angle = angle - 2 * pi;
end
while (angle < - 0.000001)
    angle = angle + 2 * pi;
end
end