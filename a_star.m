function path = a_star(grid_map, start, goal)
    % Initialize cost matrices
    [rows, cols] = size(grid_map);
    g_cost = inf(rows, cols);  % Distance from start
    f_cost = inf(rows, cols);  % g_cost + heuristic
    g_cost(start(1), start(2)) = 0;
    f_cost(start(1), start(2)) = heuristic(start, goal);

    % Open and closed lists
    open_list = [start];
    came_from = zeros(rows, cols, 2);  % Store parent nodes

    while ~isempty(open_list)
        % Find node with lowest f-cost
        [~, idx] = min(f_cost(sub2ind(size(f_cost), open_list(:, 1), open_list(:, 2))));
        current = open_list(idx, :);
        open_list(idx, :) = [];  % Remove from open list

        % Goal reached
        if isequal(current, goal)
            path = reconstruct_path(came_from, current);
            return;
        end

        % Get valid neighbors
        neighbors = get_neighbors(grid_map, current);
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            tentative_g_cost = g_cost(current(1), current(2)) + 1;  % Assume cost 1

            if tentative_g_cost < g_cost(neighbor(1), neighbor(2))
                g_cost(neighbor(1), neighbor(2)) = tentative_g_cost;
                f_cost(neighbor(1), neighbor(2)) = tentative_g_cost + heuristic(neighbor, goal);
                came_from(neighbor(1), neighbor(2), :) = current;
                if ~ismember(neighbor, open_list, 'rows')
                    open_list = [open_list; neighbor];
                end
            end
        end
    end

    path = [];  % No valid path found
end

function h = heuristic(node, goal)
    % Manhattan distance heuristic
    h = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function neighbors = get_neighbors(grid_map, node)
    % Get valid neighbors (up, down, left, right)
    directions = [0 1; 0 -1; 1 0; -1 0];
    neighbors = [];
    for i = 1:size(directions, 1)
        neighbor = node + directions(i, :);
        if neighbor(1) >= 1 && neighbor(1) <= size(grid_map, 1) && ...
           neighbor(2) >= 1 && neighbor(2) <= size(grid_map, 2) && ...
           grid_map(neighbor(1), neighbor(2)) == 0
            neighbors = [neighbors; neighbor];
        end
    end
end

function path = reconstruct_path(came_from, current)
    % Reconstruct the path by backtracking
    path = current;
    while any(came_from(current(1), current(2), :))
        current = squeeze(came_from(current(1), current(2), :))';
        path = [current; path];
    end
end
