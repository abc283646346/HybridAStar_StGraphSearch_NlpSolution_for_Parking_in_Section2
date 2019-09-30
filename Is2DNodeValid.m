function is_collision_free = Is2DNodeValid(xx0, yy0, xx, yy)
is_collision_free = 1;
global planning_scale_
global hybrid_astar_

num_sample = 4;
x_all = linspace(xx0, xx, num_sample);
y_all = linspace(yy0, yy, num_sample);
global costmap_
for ind = 1 : num_sample
    x = x_all(ind);
    y = y_all(ind);
    if ((x > planning_scale_.xmax) || (x < planning_scale_.xmin) || (y > planning_scale_.ymax) || (y < planning_scale_.ymin))
        is_collision_free = 0;
        break;
    end
    if (costmap_(round((x - planning_scale_.xmin) /  hybrid_astar_.resolution_x + 1), round((y - planning_scale_.ymin) /  hybrid_astar_.resolution_y + 1)) == 1)
        is_collision_free = 0;
        break;
    end
end