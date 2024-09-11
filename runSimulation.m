function runSimulation(algorithm)
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

    % Display the 3D scenario
    show3D(scenario);
    lightangle(-45,30);
    view(60,50);

    % Create a binary occupancy map
    map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], MapSize=[10 10], MapHeightLimits=[0 3]);
    inflate(map, 0.2);

    % Show the 2D occupancy map
    figure;
    show(map);
    title('2D Occupancy Map');

    % Define the start and goal positions as integers
    startPosition = round([0.5 0.5]);  % Convert to integers for the grid map
    goalPosition = round([7.5 7.5]);   

    % Use the appropriate algorithm for path planning
    switch algorithm
        case 'a_star'
            waypoints = a_star(int8(occupancyMatrix(map)), startPosition, goalPosition);
        case 'rrt'
            waypoints = rrt(int8(occupancyMatrix(map)), startPosition, goalPosition);
        case 'dijkstra'
            waypoints = dijkstra(int8(occupancyMatrix(map)), startPosition, goalPosition);
        otherwise
            % Default PRM algorithm if no algorithm is chosen
            numnodes = 1000;
            planner = mobileRobotPRM(map, numnodes);
            planner.ConnectionDistance = 1.0;
            waypoints = findpath(planner, startPosition, goalPosition);
    end

    if isempty(waypoints)
        disp('No valid path found.');
    else
        % Add Z-coordinate to waypoints for 3D navigation
        waypoints3DToGoal = [waypoints, zeros(size(waypoints,1),1)];

        % Generate a trajectory
        totalTime = size(waypoints, 1) * 2;
        trajToGoal = waypointTrajectory(SampleRate=10, TimeOfArrival=linspace(0, totalTime, size(waypoints,1)), Waypoints=waypoints3DToGoal, ReferenceFrame="ENU");

        % Plot the planned path in 2D
        hold on;
        plot(waypoints(:,1), waypoints(:,2), '-o', 'LineWidth', 2, 'Color', 'blue');
        title('Planned Path (2D)');
        hold off;

        % Load the robot model and set up with the trajectory
        huskyRobot = loadrobot('clearpathHusky');
        platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, BaseTrajectory=trajToGoal);

        % Visualize the robot path in 3D
        figure;
        [ax, plotFrames] = show3D(scenario);
        lightangle(-45,30);
        view(60,50);
        title('Robot Path in 3D');

        % Set up the simulation rate
        setup(scenario);
        r = rateControl(10);

        % Run the simulation loop and stop at the goal
        while advance(scenario)
            show3D(scenario, Parent=ax, FastUpdate=true);
            waitfor(r);
            currentPose = read(platform);
            if norm(currentPose(1:2) - goalPosition) < 0.1
                disp('Robot has reached the goal. Stopping simulation.');
                break;
            end
        end
    end
end
