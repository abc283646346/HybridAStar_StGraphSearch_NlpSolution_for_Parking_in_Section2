function my_cell = GenerateRandomStaticObstacles(Nobs, areaA)
global planning_scale_
global areaV
areaV = areaA;
my_cell = cell(1, Nobs);
counter = 0;
global fixed_num_vertexes
fixed_num_vertexes = 4;
while (counter < Nobs)
    geometric_center = [planning_scale_.xmin + rand * planning_scale_.xhorizon, planning_scale_.ymin + rand * planning_scale_.yhorizon];
    vertex_cell = GenerateConvexVertexes(geometric_center, fixed_num_vertexes);
    if (IsObstacleValid(vertex_cell))
        counter = counter + 1;
        my_cell{counter} = vertex_cell;
    end
end
end

function vertex_cell = GenerateConvexVertexes(geometric_center, num_vertexes)
global areaV
x = geometric_center(1) + randn(1, num_vertexes) .* 3;
y = geometric_center(2) + randn(1, num_vertexes) .* 3;
[K, A] = convhull(x, y);
while (A > areaV)
    x = geometric_center(1) + randn(1, num_vertexes) .* 3;
    y = geometric_center(2) + randn(1, num_vertexes) .* 3;
    [K, A] = convhull(x, y);
end
vertex_cell.x = x(K);
vertex_cell.y = y(K);
vertex_cell.A = A;
end

function is_valid = IsObstacleValid(V)
global fixed_num_vertexes
if (length(V.x) ~= fixed_num_vertexes + 1)
    is_valid = 0;
    return;
end
global BV_
x0 = BV_.x0;
y0 = BV_.y0;
theta0 = BV_.theta0;
xtf = BV_.xtf;
ytf = BV_.ytf;
thetatf = BV_.thetatf;
is_valid = 1;
if (IsVehiclePoseCollidingVertexes(x0,y0,theta0,V)||IsVehiclePoseCollidingVertexes(xtf,ytf,thetatf,V))
    is_valid = 0;
end
end

function is_collided = IsVehiclePoseCollidingVertexes(x0, y0, theta0, V)
is_collided = 0;
global vehicle_geometrics_
cos_theta = cos(theta0);
sin_theta = sin(theta0);
lon_lb = -vehicle_geometrics_.rear_hang;
lon_ub = vehicle_geometrics_.front_hang + vehicle_geometrics_.wheelbase;
lat_lb = -vehicle_geometrics_.width * 0.5;
lat_ub = vehicle_geometrics_.width * 0.5;

for lon = [lon_lb : 0.1 : lon_ub, lon_ub]
    for lat = [lat_lb : 0.1 : lat_ub, lat_ub]
        x = x0 + lon * cos_theta + lat * sin_theta;
        y = y0 + lon * sin_theta - lat * cos_theta;
        is_collided = inpolygon(x,y,V.x,V.y);
        if (is_collided)
            break;
        end
    end
    if (is_collided)
        break;
    end
end
end