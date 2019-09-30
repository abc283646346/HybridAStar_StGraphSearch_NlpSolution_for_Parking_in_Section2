function [x, y, theta] = SimulateForOneSecond(x0, y0, theta0, v, phy)
global vehicle_geometrics_
Nfe = 4; % Number of finite elements
tf = 1;	 % Simulate for one second
hi = tf / (Nfe - 1);
x = zeros(1,Nfe);
y = zeros(1,Nfe);
theta = zeros(1,Nfe);
theta(1) = theta0;
x(1) = x0;
y(1) = y0;
for ii = 2 : Nfe
    theta(ii) = tan(phy) * v / vehicle_geometrics_.wheelbase  * hi + theta(ii-1);
    x(ii) = cos(theta(ii)) * v * hi + x(ii-1);
    y(ii) = sin(theta(ii)) * v * hi + y(ii-1);
end