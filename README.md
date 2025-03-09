# ROS_Stage_DFS_and_Motion_Planning

For Task-1, I developed a solution where the robot explores a 4x4 grid environment in the ROS Stage using the Depth-First Search (DFS) algorithm to find rewards. The environment is modeled as a graph with cells represented by an adjacency matrix.

Environment Setup: I created a 4x4 grid where each cell is represented by a vertex containing an “id,” “visited” boolean, and “reward” integer. The visited status of each cell is updated when the robot passes through it, and if the cell contains an obstacle or has already been visited, it is marked as “visited.” There are 3 rewards randomly placed in the free cells, and each reward is indicated by a “reward” data member set to 1, while others are set to 0.

DFS Algorithm Implementation: The robot follows the DFS algorithm to explore the environment. The strategy behind DFS is to explore as far as possible along each branch before backtracking. The robot starts at the root node and searches deeper into the graph, ensuring that it visits all reachable vertices.

Neighbor Traversal: The robot visits the free neighbor cells in a clockwise order: up → right → down → left. This helps the robot efficiently explore the grid while keeping the movement strategy consistent.

Experiments: I conducted two experiments:

Experiment 1: In the first experiment, the robot was tasked with finding exactly two rewards. The search ends when two rewards are found.
Experiment 2: In the second experiment, the robot was required to find all three rewards. The search finishes when the robot locates all three rewards.
For both experiments, I randomly placed the rewards in different cells for each trial, ensuring that the robot faced a new challenge with each run. The task was demonstrated in the ROS Stage environment and the nodes were implemented using Python.

This project helped me understand how to navigate and explore environments using the DFS algorithm and how to handle dynamic grid-based environments with varying reward locations. It also enhanced my skills in using ROS Stage and Python for robot control and exploration tasks.
--------------------
For Task-2, I worked on a solution where two robots are placed in the ROS Stage environment, and the task is to navigate the robots while avoiding obstacles. This task consists of two subtasks: A. Motion Planning and B. Resetting the Robot Position.

Task A: Motion Planning
Robot Setup: The starting position of Robot-1 is given as (x=-7.5, y=-7.5) in the environment, facing upwards. I programmed Robot-1 to move in a specific pattern where it starts by moving upwards until it detects an obstacle or wall using its sensors. Upon detecting the obstacle, the robot turns left or right (based on its current position) and moves in the opposite direction (down or up) until it detects another obstacle. This motion pattern repeats until the robot reaches its destination.

Sensor Integration: I utilized the robot’s sensors to ensure it doesn't collide with obstacles. The robot should only move left or right by a fixed distance R, which I defined in a YAML file, while the vertical movement (up/down) is controlled by the robot’s sensors to stop when a wall/obstacle is detected.

YAML Configuration: I set the distance R=3 as the default value in the YAML file for horizontal movement. This value is loaded into the Python code to control how far the robot moves left or right. The robot’s vertical movement (up/down) was determined based on the distance from the nearest obstacle detected by its sensors.

Continuous Navigation: The robot's direction was always adjusted so that it faced the direction it was moving. This ensured that the motion remained smooth and predictable.

Task B: Resetting the Robot Position
Position Reset: To reset the robot's position, I created a Python node that called the /reset_positions service in ROS Stage. This node ensures that after the robot completes its navigation, it can be returned to its initial position (x=-7.5, y=-7.5) to test the navigation process again.

Demonstration: After completing both Task A and Task B, I ran the reset node in the terminal, demonstrating the robot's ability to reset and execute the navigation pattern repeatedly without issues.

Through this task, I enhanced my skills in sensor integration for robot motion, navigation in obstacle-laden environments, and using ROS services to control robot behaviors programmatically. I also gained experience with YAML configuration for robot parameters and implemented a practical motion planning solution that can be reused in similar scenarios.
-------------------------
For Task-3, I developed a solution where two robots (tb3_1 and tb3_2) are placed in a Gazebo simulation. The task involved moving tb3_1 to the initial location of tb3_2 by sending its location through a custom message, and using a service node to calculate the Euclidean distance between the two robots' positions.

Steps Taken:
Node Setup:

I started by recalling the Python node “turtlebot3_pointop_key” from the “turtlebot3_example” package, which is designed to move a robot to a goal point in Gazebo.
I created a new Python node in my own package with the same name, “turtlebot3_pointop_key”, and copied the code from the “turtlebot3_example” file to my new file.
Launch File Creation:

I created a new launch file named “turtlebot3_empy_world.launch”. This launch file uses the world file from the “turtlebot3_gazebo” package, specifically the world used in “turtlebot3_empy_world.launch”.
The launch file is configured to start two robots instead of one, named tb3_1 and tb3_2.
Robot Communication:

The goal of the task was to move tb3_1 to the initial location of tb3_2. For this, tb3_2 sends its current location (x, y) over a topic using a custom message.
I created a message type called “coordinates”, which includes two parameters: x and y, representing the robot's location. This message is published by tb3_2.
Euclidean Distance Calculation:

In the original “turtlebot3_pointop_key” file, the code directly calculates the Euclidean distance between the current position of tb3_1 and the goal position.
Instead of performing this calculation in the “turtlebot3_pointop_key” node, I wrote a service node dedicated to calculating the Euclidean distance between two points.
The “turtlebot3_pointop_key” node then acts as a client, requesting the service node to calculate the distance between tb3_1’s current position and tb3_2’s location, which is sent via the custom message.
Demonstration in Gazebo:

I ran the setup in Gazebo, where tb3_1 successfully moves to the position of tb3_2, based on the location data received from tb3_2.
The Euclidean distance is dynamically calculated by the service node, allowing tb3_1 to navigate accurately to tb3_2’s initial location.
Results and Reflection:
Through this task, I implemented communication between two robots in Gazebo using custom messages and services. I also gained experience in creating service nodes that interact with other nodes in a client-server relationship. This setup allowed for the flexible calculation of distances and smooth navigation of the robots in a simulated environment.
-----------------------
