function [final_path_x, final_path_y, distance] = GenerateAStarPath(begin_config, end_config)
global hybrid_astar_
grid_space_2D_ = cell(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y);

% Create the node w.r.t. the initial config
g = 0;
h = CalculateH_2D(begin_config);
f = g + h;
init_idx = Convert2DConfigToIndex(begin_config);
% 1x 2y 3f 4g 5h 6is_in_open 7is_in_closed 8-9parent's index
openlist_ = [begin_config, f, g, h, 0, 0, 0, 0];
grid_space_2D_{init_idx(1), init_idx(2)} = [begin_config, f, g, h, 0, 0, 0, 0];
expansion_pattern = [-1 1; -1 0; -1 -1; 0 1; 0 -1; 1 1; 1 0; 1 -1];
expansion_length = [1.414; 1; 1.414; 1; 1; 1.414; 1; 1.414];
iter = 0;
complete_flag = 0;
while ((~isempty(openlist_)) && (iter <= 500) && (~complete_flag))
    iter = iter + 1;
    % Locate the node with smallest f value in the openlist
    local_index = find(openlist_(:,3) == min(openlist_(:,3)));
    local_index = local_index(end);
    % Name it as cur_node and prepare for extension
    cur_node = openlist_(local_index, :);
    cur_config = cur_node(1:2);
    cur_index = Convert2DConfigToIndex(cur_config);
    cur_g = cur_node(4);
    cur_x = cur_node(1);
    cur_y = cur_node(2);
    % Criterion to terminate before iter == max_iter because a good solution is derived
    if (norm(cur_config(1:2) - end_config(1:2)) < hybrid_astar_.terminal_xy_neiborhood)
        complete_flag = 1;
        best_ever_index = cur_index;
        break;
    end
    % Remove cur_node from open list and add it in closed list
    openlist_(local_index, :) = [];
    grid_space_2D_{cur_index(1), cur_index(2)}(6) = 0;
    grid_space_2D_{cur_index(1), cur_index(2)}(7) = 1;
    
    for ii = 1 : length(expansion_pattern)
        child_node_config = [cur_x, cur_y] + expansion_pattern(ii,:);
        child_node_index = Convert2DConfigToIndex(child_node_config);
        child_g = cur_g + expansion_length(ii);
        child_h = CalculateH_2D(child_node_config);
        child_f = child_g + child_h;
        child_node_prepare = [child_node_config, child_f, child_g, child_h, 1, 0, cur_index];
        % If the child node has been explored ever before
        if (~isempty(grid_space_2D_{child_node_index(1), child_node_index(2)}))
            % If the child has been within the closed list, abandon it and continue.
            if (grid_space_2D_{child_node_index(1), child_node_index(2)}(7) == 1)
                continue;
            end
            % The child must be in the open list now, then check if its
            % recorded parent deserves to be switched as our cur_node.
            if (grid_space_2D_{child_node_index(1), child_node_index(2)}(8) > child_g)
                local_index = find(openlist_(:,3) == grid_space_2D_{child_node_index(1), child_node_index(2)}(3));
                openlist_(local_index, :) = [];
                grid_space_2D_{child_node_index(1), child_node_index(2)} = child_node_prepare;
                openlist_ = [openlist_; child_node_prepare];
            end
        else
            % Child node has never been explored before
            if (Is2DNodeValid(cur_x, cur_y, child_node_config(1), child_node_config(2)))
                % If the child node is collison free
                grid_space_2D_{child_node_index(1), child_node_index(2)} = child_node_prepare;
                openlist_ = [openlist_; child_node_prepare];
            else
                % If the child node involves collisons
                child_node_prepare(7) = 1;
                child_node_prepare(6) = 0;
                grid_space_2D_{child_node_index(1), child_node_index(2)} = child_node_prepare;
            end
        end
    end
end

final_path_x = [];
final_path_y = [];
distance = 0;
if (complete_flag)
    % Derive hybrid A* result
    cur_best_parent_index = grid_space_2D_{best_ever_index(1), best_ever_index(2)}(8:9);
    final_path_x = [grid_space_2D_{best_ever_index(1), best_ever_index(2)}(1)];
    final_path_y = [grid_space_2D_{best_ever_index(1), best_ever_index(2)}(2)];
    while (norm(cur_best_parent_index) > 0)
        cur_node = grid_space_2D_{cur_best_parent_index(1), cur_best_parent_index(2)};
        cur_best_parent_index = cur_node(8:9);
        final_path_x = [cur_node(1), final_path_x];
        final_path_y = [cur_node(2), final_path_y];
    end
    for ii = 2 : length(final_path_x)
        distance = distance + hypot(final_path_x(ii) - final_path_x(ii-1), final_path_y(ii) - final_path_y(ii-1));
    end
end
end

function idx = Convert2DConfigToIndex(config)
global hybrid_astar_ planning_scale_
ind1 = ceil((config(1) - planning_scale_.xmin) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((config(2) - planning_scale_.ymin) / hybrid_astar_.resolution_y) + 1;

if (ind1 > hybrid_astar_.num_nodes_x)
    ind1 = hybrid_astar_.num_nodes_x;
elseif (ind1 < 1)
    ind1 = 1;
end
if (ind2 > hybrid_astar_.num_nodes_y)
    ind2 = hybrid_astar_.num_nodes_y;
elseif (ind2 < 1)
    ind2 = 1;
end
idx = [ind1, ind2];
end

function val = CalculateH_2D(config)
global hybrid_astar_ BV_
val = hybrid_astar_.multiplier_H * sum(abs(config - [BV_.xtf, BV_.ytf]));
end