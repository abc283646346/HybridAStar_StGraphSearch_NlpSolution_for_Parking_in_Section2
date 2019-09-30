function [x, y, theta, path_length] = GenerateRsPath(startPose, goalPose, output_path_resolution)
global vehicle_physics_ hybrid_astar_
reedsConnObj = robotics.ReedsSheppConnection('MinTurningRadius', vehicle_physics_.turning_radius_min);
reedsConnObj.ReverseCost = hybrid_astar_.penalty_multiplier_for_reversing;
[pathSegObj,~] = connect(reedsConnObj,startPose,goalPose);
path_length = pathSegObj{1}.Length;
if (output_path_resolution > 0)
    poses = interpolate(pathSegObj{1},[0:output_path_resolution:path_length]);
    x = poses(:,1);
    y = poses(:,2);
    theta = poses(:,3);
else
    x = [];
    y = [];
    theta = [];
end