function pathLength = calculatePathLength(path)
    % Function to calculate the path length
    pathLength = 0;
    for i = 1:size(path, 1) - 1
        dist = sqrt((path(i + 1, 1) - path(i, 1))^2 + (path(i + 1, 2) - path(i, 2))^2);
        pathLength = pathLength + dist;
    end
end
