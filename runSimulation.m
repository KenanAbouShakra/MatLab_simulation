function runSimulation()
    % Initialize a robot scenario with an update rate of 5 Hz
    scenario = robotScenario(UpdateRate=5);

    % Add a plane to represent the ground
    addMesh(scenario, "Plane", Position=[5 5 0], Size=[10 10], Color=[0.6 0.6 0.6]);

    % Add obstacles in the form of static shelves and walls
    addMesh(scenario, "Box", Position=[2 2 1], Size=[0.5 2 2], Color=[0.8 0.2 0.2], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[4 2 1], Size=[0.5 2 2], Color=[0.8 0.2 0.2], IsBinaryOccupied=true);

    % Add more walls for a challenging environment
    addMesh(scenario, "Box", Position=[6 6 1], Size=[1 0.5 2], Color=[0.9 0.5 0.1], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[8 4 1], Size=[0.5 3 2], Color=[0.2 0.8 0.8], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[3 7 1], Size=[1 0.5 2], Color=[0.9 0.2 0.5], IsBinaryOccupied=true);

    % Add a loading point to mark the goal
    addMesh(scenario, "Box", Position=[8 8 0.5], Size=[0.5 0.5 0.5], Color=[0 0.8 0]);

    % Display the 3D scenario with proper lighting and camera angles
    show3D(scenario);
    lightangle(-45,30);
    view(60,50);

    % Create a binary occupancy map to define the navigable space
    map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], MapSize=[10 10], MapHeightLimits=[0 3]);

    % Inflate the map to account for the robot's size to avoid collisions
    inflate(map, 0.2);

    % Show the 2D occupancy map for better visualization of the environment
    figure;
    show(map);
    title('2D Occupancy Map');

    % Define the start and goal positions for the robot
    startPosition = [0.5 0.5];  % Starting position away from obstacles
    goalPosition = [7.5 7.5];   % Goal position away from obstacles

    % Check if start or goal positions are occupied by obstacles
    if checkOccupancy(map, startPosition) || checkOccupancy(map, goalPosition)
        disp('Start or goal position is in an occupied area.');
        return;
    end

    % Create a probabilistic roadmap (PRM) planner to navigate the map
    numnodes = 1000;  % Number of nodes in the roadmap
    planner = mobileRobotPRM(map, numnodes);
    planner.ConnectionDistance = 1.0;  % Set the distance between connected nodes

    % Plan a path between the start and goal positions
    waypoints = findpath(planner, startPosition, goalPosition);

    if isempty(waypoints)
        disp('No valid path found. Try different start or goal positions.');
    else
        % Add a Z-coordinate (height) to the waypoints for 3D navigation
        waypoints3DToGoal = [waypoints, zeros(size(waypoints,1),1)];

        % Generate a trajectory for the robot to follow using the waypoints
        totalTime = size(waypoints, 1) * 2;  % Adjust total time for faster movement
        trajToGoal = waypointTrajectory(SampleRate=10, TimeOfArrival=linspace(0, totalTime, size(waypoints,1)), Waypoints=waypoints3DToGoal, ReferenceFrame="ENU");

        % Plot the planned path in 2D for visualization
        hold on;
        plot(waypoints(:,1), waypoints(:,2), '-o', 'LineWidth', 2, 'Color', 'blue');
        title('Planned Path (2D)');
        hold off;

        % Load the robot model and set it up with the generated trajectory
        huskyRobot = loadrobot('clearpathHusky');
        platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, BaseTrajectory=trajToGoal);

        % Visualize the robot following the path in 3D
        figure;
        [ax, plotFrames] = show3D(scenario);
        lightangle(-45,30);
        view(60,50);
        title('Robot Path in 3D');

        % Set up the simulation rate
        setup(scenario);
        r = rateControl(10);  % Adjust simulation speed

        % Run the simulation loop and stop when the robot reaches the goal
        while advance(scenario)
            show3D(scenario, Parent=ax, FastUpdate=true);
            waitfor(r);

            % Get the robot's current position
            currentPose = read(platform);

            % Stop the simulation when the robot is close to the goal
            if norm(currentPose(1:2) - goalPosition) < 0.1  % Stopping threshold
                disp('Robot has reached the goal. Stopping simulation.');
                break;  % End simulation
            end
        end
    end
end
