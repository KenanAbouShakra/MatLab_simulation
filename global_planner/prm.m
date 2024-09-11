function waypoints = prm(map, startPosition, goalPosition)
    % PRM (Probabilistic Roadmap) Path Planning Algorithm

    % Create the PRM planner with 1000 nodes
    numnodes = 1000;
    planner = mobileRobotPRM(map, numnodes);

    % Set connection distance (maximum distance between connected nodes)
    planner.ConnectionDistance = 1.0;

    % Find the path using PRM
    waypoints = findpath(planner, startPosition, goalPosition);
end
