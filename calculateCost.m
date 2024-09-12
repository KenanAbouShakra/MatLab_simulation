function cost = calculateCost(path)
    % Function to calculate the total cost (Euclidean distance) of a path
    cost = 0;
    for i = 1:size(path, 1) - 1
        dist = sqrt((path(i + 1, 1) - path(i, 1))^2 + (path(i + 1, 2) - path(i, 2))^2);
        cost = cost + dist;
    end
end
