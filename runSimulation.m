function runSimulation(algorithm)
    % Add global planner folder to the MATLAB path if it exists
    globalPlannerPath = 'global_planner';
    if isfolder(globalPlannerPath)
        addpath(globalPlannerPath);
    else
        warning('Global planner folder not found.');
    end

    % Initialize a robot scenario with an update rate of 5 Hz
    scenario = robotScenario(UpdateRate=5);

    % Add a plane to represent the ground surface with a light grey color
    addMesh(scenario, "Plane", Position=[5 5 0], Size=[10 10], Color=[0.9 0.9 0.9]);

    % Add static obstacles to the environment
    addMesh(scenario, "Box", Position=[2 2 1], Size=[0.5 2 2], Color=[0.8 0.2 0.2], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[4 2 1], Size=[0.5 2 2], Color=[0.8 0.2 0.2], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[6 6 1], Size=[1 0.5 2], Color=[0.9 0.5 0.1], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[8 4 1], Size=[0.5 3 2], Color=[0.2 0.8 0.8], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[3 7 1], Size=[1 0.5 2], Color=[0.9 0.2 0.5], IsBinaryOccupied=true);

    % Add a goal point in the environment
    addMesh(scenario, "Box", Position=[8 8 0.5], Size=[0.5 0.5 0.5], Color=[0 0.8 0]);

    % Display the 3D scenario
    [ax3D, plotFrames] = show3D(scenario);
    lightangle(-45, 30);  
    view(60, 50);        

    % Create a binary occupancy map from the scenario
    map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], MapSize=[10 10], MapHeightLimits=[0 3]);
    inflate(map, 0.2);  

    % Define the start and goal positions on the grid
    startPosition = round([0.5 0.5]);  % Round start position to ensure valid grid index
    goalPosition = round([7.5 7.5]);   % Round goal position

    % Ensure startPosition and goalPosition are positive integers and within bounds
    mapSize = size(occupancyMatrix(map));
    startPosition = max([1, 1], min(mapSize, startPosition));
    goalPosition = max([1, 1], min(mapSize, goalPosition));

    % Select the global path-planning algorithm
    switch algorithm
        case 'a_star'
            waypoints = a_star(int8(occupancyMatrix(map)), startPosition, goalPosition);
        case 'dijkstra'
            waypoints = dijkstra(int8(occupancyMatrix(map)), startPosition, goalPosition);
        case 'prm'
            waypoints = prm(map, startPosition, goalPosition);
        otherwise
            error('Unknown global algorithm. Choose "a_star", "dijkstra", or "prm".');
    end

    % Check if a valid path was found
    if isempty(waypoints)
        disp('No valid path found.');
    else
        % Add a Z-coordinate (0) to the waypoints for 3D navigation
        waypoints3DToGoal = [waypoints, zeros(size(waypoints, 1), 1)];

        % Generate a trajectory for the robot to follow based on the waypoints
        totalTime = size(waypoints, 1) * 2; 
        trajToGoal = waypointTrajectory(SampleRate=10, TimeOfArrival=linspace(0, totalTime, size(waypoints, 1)), Waypoints=waypoints3DToGoal, ReferenceFrame="ENU");

        % Plot the occupancy map with obstacles and path in 2D
        figure;
        show(map);  
        hold on;
        plot(waypoints(:,1), waypoints(:,2), '-o', 'LineWidth', 2, 'Color', 'blue');  
        text(startPosition(1), startPosition(2), 'Start', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');  
        text(goalPosition(1), goalPosition(2), 'End', 'FontSize', 12, 'Color', 'blue', 'FontWeight', 'bold');  
        title('Planned Path with Obstacles (2D)');
        xlabel('X Position');
        ylabel('Y Position');
        xlim([0 10]); 
        ylim([0 10]);  
        grid on;      

        % Add the robot as a marker in the 2D plot
        hRobot2D = plot(startPosition(1), startPosition(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red'); 
        hold off;

        % Load the Husky robot model and associate it with the trajectory
        huskyRobot = loadrobot('clearpathHusky');
        platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, BaseTrajectory=trajToGoal);

        % Visualize the robot's path in 3D
        figure;
        [ax3D, plotFrames] = show3D(scenario);
        lightangle(-45,30); 
        view(60,50);        
        title('Robot Path in 3D');

        % Plot the 3D path of the robot
        hold on;
        plot3(waypoints3DToGoal(:,1), waypoints3DToGoal(:,2), waypoints3DToGoal(:,3), '-o', 'LineWidth', 2, 'Color', 'green');
        hold off;

        % Set up the simulation rate to control the update speed
        setup(scenario);
        r = rateControl(10);

        % Run the simulation loop for both 3D and 2D display
        while advance(scenario)
            % Update 2D plot with the robot's current position
            currentPose = read(platform);
            set(hRobot2D, 'XData', currentPose(1), 'YData', currentPose(2));  % Update robot position in 2D plot
            drawnow;

            % Update the 3D view
            show3D(scenario, Parent=ax3D, FastUpdate=true); 
            waitfor(r);  

            % Check if the robot reached the goal
            if norm(currentPose(1:2) - goalPosition) < 0.1  
                disp('Robot has reached the goal. Stopping simulation.');
                break;
            end
        end
    end
end

% Function to calculate the total cost (Euclidean distance) of a path
function cost = calculateCost(path)
    cost = 0;
    for i = 1:size(path, 1) - 1
        dist = sqrt((path(i + 1, 1) - path(i, 1))^2 + (path(i + 1, 2) - path(i, 2))^2);
        cost = cost + dist;
    end
end

% Function to calculate the path length
function pathLength = calculatePathLength(path)
    pathLength = 0;
    for i = 1:size(path, 1) - 1
        dist = sqrt((path(i + 1, 1) - path(i, 1))^2 + (path(i + 1, 2) - path(i, 2))^2);
        pathLength = pathLength + dist;
    end
end
