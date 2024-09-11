function path = dijkstra(grid_map, start, goal)
    [rows, cols] = size(grid_map);
    dist = inf(rows, cols);  % Distance to start node
    dist(start(1), start(2)) = 0;
    prev = zeros(rows, cols, 2);  % Store previous nodes
    visited = false(rows, cols);  % Keep track of visited nodes
    
    % Priority queue for unvisited nodes (row, col, distance)
    pq = [start, 0];
    
    while ~isempty(pq)
        % Sort the queue by distance and pop the closest node
        pq = sortrows(pq, 3);
        current = pq(1, 1:2);
        pq(1, :) = [];
        
        if visited(current(1), current(2))
            continue;
        end
        
        % Mark the current node as visited
        visited(current(1), current(2)) = true;
        
        % If the goal is reached, reconstruct the path
        if isequal(current, goal)
            path = reconstruct_path(prev, start, goal);
            return;
        end
        
        % Get neighbors of the current node
        neighbors = get_neighbors(grid_map, current);
        
        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            if visited(neighbor(1), neighbor(2))
                continue;
            end
            
            % Calculate tentative distance to neighbor
            tentative_dist = dist(current(1), current(2)) + 1;  % Assuming grid movement cost is 1
            
            if tentative_dist < dist(neighbor(1), neighbor(2))
                dist(neighbor(1), neighbor(2)) = tentative_dist;
                prev(neighbor(1), neighbor(2), :) = current;
                pq = [pq; neighbor, tentative_dist];
            end
        end
    end
    
    path = [];  % If no path found
end

function path = reconstruct_path(prev, start, goal)
    path = [goal];
    current = goal;
    
    while ~isequal(current, start)
        current = squeeze(prev(current(1), current(2), :))';
        path = [current; path];
    end
end

function neighbors = get_neighbors(grid_map, node)
    % Get valid neighbors in the 4 cardinal directions (up, down, left, right)
    neighbors = [];
    directions = [0 1; 0 -1; 1 0; -1 0];  % Up, Down, Right, Left
    
    for i = 1:size(directions, 1)
        neighbor = node + directions(i, :);
        if neighbor(1) >= 1 && neighbor(1) <= size(grid_map, 1) && ...
           neighbor(2) >= 1 && neighbor(2) <= size(grid_map, 2) && ...
           grid_map(neighbor(1), neighbor(2)) == 0  % Ensure not in obstacle
            neighbors = [neighbors; neighbor];
        end
    end
end
