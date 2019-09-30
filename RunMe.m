% ==============================================================================
% MATLAB Source Codes for Book "Cooperative decision-making and trajectory
% planning for CAVs" to be published in 2020. Copyright (C) 2019 Bai Li
% ==============================================================================
%   In addition to citing the book's related chapers, users must cite ALL of the
%   following article when utilizing these source codes to produce new
%   papers:
%   (1) Li, B., & Shao, Z. (2015). A unified motion planning method for parking
%   an autonomous vehicle in the presence of irregularly placed obstacles.
%   Knowledge-Based Systems, 86, 11-20.
%   (2) Li, B., & Shao, Z. (2015). Simultaneous dynamic optimization: A
%   trajectory planning method for nonholonomic car-like robots. Advances
%   in Engineering Software, 87, 30-42.
%   (3) Li, B., Wang, K., & Shao, Z. (2016). Time-optimal maneuver planning in
%   automatic parallel parking using a simultaneous dynamic optimization
%   approach. IEEE Transactions on Intelligent Transportation Systems,
%   17(11), 3263-3274.
%   (4) Li, B., & Zhang, Y. (2019, July). Fast Trajectory Planning for Off-Road
%   Autonomous Driving with a Spatiotemporal Tunnel and Numerical Optimal
%   Control Approach. In 2019 IEEE 4th International Conference on Advanced
%   Robotics and Mechatronics (ICARM) (pp. 924-929). IEEE.
% ==============================================================================
% If there are inquiries, contact libai@zju.edu.cn
% 2019.09.29
% ==============================================================================
clear all
close all
clc

% Basic parameter setting
InitParams;
% Specify starting and goal configs
global BV_
BV_.x0 = -10;
BV_.y0 = 0;
BV_.theta0 = 0;
BV_.v0 = 0;
BV_.a0 = 0;
BV_.phy0 = 0;
BV_.w0 = 0;
BV_.xtf = 8;
BV_.ytf = 0;
BV_.thetatf = pi / 2;
BV_.vtf = 0;
BV_.atf = 0;
BV_.phytf = 0;
BV_.wtf = 0;

global num_static_obs num_dynamic_obs
num_static_obs = 4;
num_dynamic_obs = 8;
Nfe = 101;
global norm_tf
norm_tf = 40;
global obstacle_vertexes_ 
obstacle_vertexes_ = GenerateRandomStaticObstacles(num_static_obs, 25);
global moving_obstacle_vertexes_
moving_obstacle_vertexes_ = GenerateRandomStaticObstacles(num_dynamic_obs, 3);
global obstacle_frame_x_ obstacle_frame_y_
[obstacle_frame_x_, obstacle_frame_y_] = GenerateWholeObstacles(Nfe, obstacle_vertexes_, moving_obstacle_vertexes_);
global costmap_
costmap_ = CreateDilatedCostmap();
WriteFilesForNLP(Nfe, num_static_obs, num_dynamic_obs);

global hybrid_x hybrid_y
[x, y, theta] = PlanHybridAStarPath();
[hybrid_x, hybrid_y, hybrid_theta, ~] = ResampleConfig(x, y, theta, theta, Nfe);

[hybrid_x, hybrid_y, hybrid_theta] = SearchVelocityInStGraph(hybrid_x, hybrid_y, hybrid_theta, norm_tf);

WriteInitialGuessForNLP(hybrid_x, hybrid_y, hybrid_theta);
!ampl r1.run
% Solution illustration
ProduceVideo;