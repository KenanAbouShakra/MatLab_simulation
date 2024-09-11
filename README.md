# MatLab_simulation

## Motion planning simulation

### Objective:
The goal of this project is to simulate the motion planning of a mobile robot navigating through an environment filled with obstacles such as walls and shelves. The robot's goal is to travel from a start position to a designated goal without collisions. We explored three different path planning algorithms: PRM, A*, and Dijkstra, each applied to this environment to observe how they handle pathfinding under constraints.

### Environment Setup:
- **Ground Plane**: Flat surface on which the robot navigates.
- **Obstacles**: Static walls and shelves.
- **Binary Occupancy Map**: Used to represent the navigable space and obstacles.
- **Loading Point**: The robot’s goal.

### Path Planning Algorithms:

#### A* Algorithm:
- **Description**: A* is a graph-based search algorithm that uses a combination of the shortest known path (g-cost) and a heuristic estimate (h-cost) to find the optimal path.
- **Strengths**: Efficient in finding the shortest path and uses heuristics for faster convergence.
- **Outcome**: A* successfully found a collision-free path by efficiently evaluating each node's cost.
- **2D Path**: ![A* 2D](./resources/images/a_star2D.png)
- **3D Path**: ![A* 3D](./resources/images/a_star3D.png)

#### Dijkstra’s Algorithm:
- **Description**: Dijkstra’s algorithm finds the shortest path by exploring all possible routes from the start to the goal node, assigning each node a distance value and choosing the smallest distance until the goal is reached.
- **Strengths**: Guarantees the shortest path, though it can be slower than heuristic-based methods.
- **Outcome**: Dijkstra’s algorithm found the optimal path by checking all possible nodes, though it was slower than A*.
- **2D Path**: ![Dijkstra 2D](./resources/images/dijkstra2D.png)

#### Probabilistic Roadmap (PRM):
- **Description**: PRM is a sampling-based algorithm that randomly generates nodes in the environment and connects them to form a roadmap. The robot then follows the roadmap from start to goal.
- **Strengths**: Works well in high-dimensional spaces and environments with many obstacles.
- **Outcome**: PRM successfully generated a roadmap, but its performance is highly dependent on the number of nodes and the connection distance.
- **2D Path**: ![PRM 2D](./resources/images/prm2D.png)
- **3D Path**: ![PRM 3D](./resources/images/prm3D.png)

### Visualization:
- **2D Map**: Shows the occupancy grid and the robot’s movement plan for each algorithm.
- **3D View**: Provides real-time navigation and visualization of the robot’s trajectory through the environment.

### Simulation Dynamics:
- The simulation runs at a frequency of 10 Hz.
- The robot's speed and movement can be adjusted.
- The simulation ends once the robot reaches the goal, based on a proximity threshold.

### Conclusion:
This simulation highlights how different path planning algorithms perform in a constrained environment. A* and Dijkstra successfully find the shortest paths, with A* being faster due to its heuristic, while PRM is useful in environments where sampling methods are preferred but requires careful parameter tuning to ensure success.

This project demonstrates the strengths and limitations of each algorithm in terms of efficiency, pathfinding accuracy, and how they deal with obstacles.
