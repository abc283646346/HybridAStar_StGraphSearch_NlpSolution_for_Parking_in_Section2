global planning_scale_
planning_scale_.xmin = -20;
planning_scale_.xmax = 20;
planning_scale_.ymin = -20;
planning_scale_.ymax = 20;
planning_scale_.xhorizon = 40;
planning_scale_.yhorizon = 40;

global vehicle_geometrics_
vehicle_geometrics_.wheelbase = 2.8;
vehicle_geometrics_.front_hang = 0.96;
vehicle_geometrics_.rear_hang = 0.929;
vehicle_geometrics_.width = 1.942;
vehicle_geometrics_.length = vehicle_geometrics_.wheelbase + vehicle_geometrics_.front_hang + vehicle_geometrics_.rear_hang;
vehicle_geometrics_.radius = hypot(0.25 * vehicle_geometrics_.length, 0.5 * vehicle_geometrics_.width);
vehicle_geometrics_.r2x = 0.25 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;
vehicle_geometrics_.f2x = 0.75 * vehicle_geometrics_.length - vehicle_geometrics_.rear_hang;

global vehicle_physics_
vehicle_physics_.v_max = 2.5;
vehicle_physics_.a_max = 0.5;
vehicle_physics_.phy_max = 0.7;
vehicle_physics_.w_max = 0.5;
vehicle_physics_.kappa_max = tan(vehicle_physics_.phy_max) / vehicle_geometrics_.wheelbase;
vehicle_physics_.turning_radius_min = vehicle_geometrics_.wheelbase / tan(vehicle_physics_.phy_max);

global hybrid_astar_
hybrid_astar_.resolution_x = 0.1;
hybrid_astar_.resolution_y = 0.1;
hybrid_astar_.resolution_theta = 0.1;
hybrid_astar_.num_nodes_x = ceil(planning_scale_.xhorizon / hybrid_astar_.resolution_x) + 1;
hybrid_astar_.num_nodes_y = ceil(planning_scale_.yhorizon / hybrid_astar_.resolution_x) + 1;
hybrid_astar_.num_nodes_theta = ceil(2 * pi / hybrid_astar_.resolution_theta) + 1;
hybrid_astar_.penalty_multiplier_for_reversing = 3;
hybrid_astar_.penalty_multiplier_for_drastic_direction_change = 5;
hybrid_astar_.multiplier_H = 3.0;
hybrid_astar_.num_iters_for_rs = 20;
hybrid_astar_.max_iter = 500;
hybrid_astar_.terminal_xy_neiborhood = 0.5;
hybrid_astar_.terminal_theta_neiborhood = 0.2;