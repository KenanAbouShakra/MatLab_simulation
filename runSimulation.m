function runSimulation(algorithm)
    % Initialize a robot scenario with an update rate of 5 Hz.
    scenario = robotScenario(UpdateRate=5);

    % Add a plane to represent the ground surface.
    addMesh(scenario, "Plane", Position=[5 5 0], Size=[10 10], Color=[0.6 0.6 0.6]);

    % Add obstacles to the environment. These obstacles will be modeled as static boxes (shelves or walls).
    addMesh(scenario, "Box", Position=[2 2 1], Size=[0.5 2 2], Color=[0.8 0.2 0.2], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[4 2 1], Size=[0.5 2 2], Color=[0.8 0.2 0.2], IsBinaryOccupied=true);

    % Add more walls to create a challenging environment for the robot.
    addMesh(scenario, "Box", Position=[6 6 1], Size=[1 0.5 2], Color=[0.9 0.5 0.1], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[8 4 1], Size=[0.5 3 2], Color=[0.2 0.8 0.8], IsBinaryOccupied=true);
    addMesh(scenario, "Box", Position=[3 7 1], Size=[1 0.5 2], Color=[0.9 0.2 0.5], IsBinaryOccupied=true);

    % Add a goal point in the environment, represented by a small box.
    addMesh(scenario, "Box", Position=[8 8 0.5], Size=[0.5 0.5 0.5], Color=[0 0.8 0]);

    % Display the 3D scenario that has been set up so far.
    [ax, plotFrames] = show3D(scenario);
    lightangle(-45,30);  
    view(60,50);        

    % Create a binary occupancy map from the scenario, defining the size and height limits.
    map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], MapSize=[10 10], MapHeightLimits=[0 3]);
    inflate(map, 0.2);  

    % Define the start and goal positions on the grid as integers.
    startPosition = round([0.5 0.5]);  
    goalPosition = round([7.5 7.5]);  

    % Select the path-planning algorithm based on user input.
    switch algorithm
        case 'a_star'
            waypoints = a_star(int8(occupancyMatrix(map)), startPosition, goalPosition);
        case 'rrt'
            waypoints = rrt(int8(occupancyMatrix(map)), startPosition, goalPosition);
        case 'dijkstra'
            waypoints = dijkstra(int8(occupancyMatrix(map)), startPosition, goalPosition);
        otherwise
            % Use PRM (Probabilistic Roadmap) as the default algorithm if none is specified.
            numnodes = 1000; 
            planner = mobileRobotPRM(map, numnodes);
            planner.ConnectionDistance = 1.0;  
            waypoints = findpath(planner, startPosition, goalPosition);
    end

    % Check if a valid path was found. If not, display a message and exit.
    if isempty(waypoints)
        disp('No valid path found.');
    else
        % Add a Z-coordinate (0) to the waypoints for 3D navigation.
        waypoints3DToGoal = [waypoints, zeros(size(waypoints,1),1)];

        % Generate a trajectory for the robot to follow based on the waypoints.
        totalTime = size(waypoints, 1) * 2; 
        trajToGoal = waypointTrajectory(SampleRate=10, TimeOfArrival=linspace(0, totalTime, size(waypoints,1)), Waypoints=waypoints3DToGoal, ReferenceFrame="ENU");

        % Plot the occupancy map, including the obstacles and path in 2D
        figure;
        show(map);  
        hold on;
        plot(waypoints(:,1), waypoints(:,2), '-o', 'LineWidth', 2, 'Color', 'blue');  % Plot the planned path
        text(startPosition(1), startPosition(2), 'Start', 'FontSize', 12, 'Color', 'red', 'FontWeight', 'bold');  % Mark start.
        text(goalPosition(1), goalPosition(2), 'End', 'FontSize', 12, 'Color', 'blue', 'FontWeight', 'bold');  % Mark goal.
        title('Planned Path with Obstacles (2D)');
        xlabel('X Position');
        ylabel('Y Position');
        xlim([0 10]); 
        ylim([0 10]);  
        grid on;      
        hold off;

        % Load the Husky robot model and associate it with the trajectory.
        huskyRobot = loadrobot('clearpathHusky');
        platform = robotPlatform("husky", scenario, RigidBodyTree=huskyRobot, BaseTrajectory=trajToGoal);

        % Visualize the robot's path in 3D.
        figure;
        [ax, plotFrames] = show3D(scenario);
        lightangle(-45,30); 
        view(60,50);        
        title('Robot Path in 3D');

        % Plot the 3D path of the robot in a distinct color (green).
        hold on;
        plot3(waypoints3DToGoal(:,1), waypoints3DToGoal(:,2), waypoints3DToGoal(:,3), '-o', 'LineWidth', 2, 'Color', 'green');
        hold off;

        % Set up the simulation rate to control the update speed.
        setup(scenario);
        r = rateControl(10);

        % Run the simulation loop and stop when the robot reaches the goal.
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
