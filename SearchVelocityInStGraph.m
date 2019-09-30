function [xx, yy, tt] = SearchVelocityInStGraph(hybrid_x, hybrid_y, hybrid_theta, norm_tf)
nfe = length(hybrid_x);
s = 0;
for ii = 2 : nfe
    s = s + hypot(hybrid_x(ii) - hybrid_x(ii-1), hybrid_y(ii) - hybrid_y(ii-1));
end
st_gridmap = zeros(nfe, nfe);

global obstacle_frame_x_ obstacle_frame_y_
global num_static_obs num_dynamic_obs
for ii = 1 : nfe % x axis (time)
    for jj = 1 : nfe % y axis (travel distance)
        x = hybrid_x(jj);
        y = hybrid_y(jj);
        theta = hybrid_theta(jj);
        for kk = (num_static_obs + 1) : (num_static_obs + num_dynamic_obs)
            obs_x = obstacle_frame_x_(ii, kk, :);
            obs_y = obstacle_frame_y_(ii, kk, :);
            if (IsVehicleCollidingWithMovingObstacle(x,y,theta,obs_x,obs_y))
                st_gridmap(ii, jj) = 1;
                continue;
            end
        end
    end
end

[st_x, st_y] = SearchStPathViaAStar(st_gridmap);
if (length(st_x) > 0)
    index_vector = RegularizeStSearchResult(st_x, st_y, nfe);
    xx = [];
    yy = [];
    tt = [];
    for ii = 1 : nfe
        index = index_vector(ii);
        xx = [xx, hybrid_x(index)];
        yy = [yy, hybrid_y(index)];
        tt = [tt, hybrid_theta(index)];
    end
else
    disp('[SearchVelocityInStGraph] ST graph search failed.');
    xx = hybrid_x; yy = hybrid_y; tt = hybrid_theta;
end
end

function y = RegularizeStSearchResult(st_x, st_y, nfe)
y = [st_y(1,1)];
for ii = 2 : length(st_x)
    if (st_x(ii) ~= st_x(ii-1))
        y = [y, st_y(ii)];
    end
end
if (length(y) ~= nfe)
    error 1;
end
end