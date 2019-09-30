function [xx,yy] = GenerateWholeObstacles(Nfe, obstacle_vertexes_, moving_obstacle_vertexes_)
Nstatic = length(obstacle_vertexes_);
Ndynamic = length(moving_obstacle_vertexes_);
Nobs = Nstatic + Ndynamic;
xx = zeros(Nfe, Nobs, 4);
yy = zeros(Nfe, Nobs, 4);

for ii = 1 : Nstatic
    vec_x = obstacle_vertexes_{ii}.x;
    vec_y = obstacle_vertexes_{ii}.y;
    for jj = 1 : Nfe
        for kk = 1 : 4
            xx(jj,ii,kk) = vec_x(1,kk);
            yy(jj,ii,kk) = vec_y(1,kk);
        end
    end
end

for ii = 1 : Ndynamic
    vec_x = moving_obstacle_vertexes_{ii}.x;
    vec_y = moving_obstacle_vertexes_{ii}.y;
    shift_x = randn * 5 / Nfe;
    shift_y = randn * 5 / Nfe;
    for jj = 1 : Nfe
        for kk = 1 : 4
            xx(jj,ii+Nstatic,kk) = vec_x(1,kk) + shift_x * jj;
            yy(jj,ii+Nstatic,kk) = vec_y(1,kk) + shift_y * jj;
        end
    end
end
end