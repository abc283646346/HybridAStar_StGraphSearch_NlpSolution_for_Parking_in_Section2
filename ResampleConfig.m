function [xf, yf, thetaf, phyf] = ResampleConfig(x, y, theta, phy, NE)

for ii = 2 : length(theta)
    while (theta(ii) - theta(ii-1) > pi)
        theta(ii) = theta(ii) - 2 * pi;
    end
    while (theta(ii) - theta(ii-1) < -pi)
        theta(ii) = theta(ii) + 2 * pi;
    end
end

ex_x = [];
ex_y = [];
ex_t = [];
ex_p = [];

for ii = 1 : (length(x) - 1)
    distance = hypot(x(ii+1)-x(ii),y(ii+1)-y(ii));
    LARGE_NUM = round(distance * 100);
    temp = linspace(x(ii), x(ii+1), LARGE_NUM);
    temp = temp(1,1:(LARGE_NUM - 1));
    ex_x = [ex_x, temp];
    
    temp = linspace(y(ii), y(ii+1), LARGE_NUM);
    temp = temp(1,1:(LARGE_NUM - 1));
    ex_y = [ex_y, temp];
    
    temp = linspace(theta(ii), theta(ii+1), LARGE_NUM);
    temp = temp(1,1:(LARGE_NUM - 1));
    ex_t = [ex_t, temp];
    
    temp = linspace(phy(ii), phy(ii+1), LARGE_NUM);
    temp = temp(1,1:(LARGE_NUM - 1));
    ex_p = [ex_p, temp];
end
ex_x = [ex_x, x(end)];
ex_y = [ex_y, y(end)];
ex_t = [ex_t, theta(end)];
ex_p = [ex_p, phy(end)];

Len = length(ex_x);
index = round(linspace(1, Len, NE));
xf = ex_x(index);
yf = ex_y(index);
thetaf = ex_t(index);
phyf = ex_p(index);