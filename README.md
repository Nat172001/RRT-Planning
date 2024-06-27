# RRT-Planning
This project implements the Rapidly-exploring Random Tree (RRT) algorithm for path planning in a grid-based environment. The algorithm generates a tree of possible paths from a start point to a goal point, navigating around obstacles. The implementation includes ROS 2 integration to handle robot start and goal positions via topic subscriptions and visualize the environment and path using markers and occupancy grids. The grid is defined with specific boundaries and size, and a utility function converts poses to grid coordinates and vice versa. 



https://github.com/Nat172001/RRT-Planning/assets/119772443/f3587057-5984-4347-99f3-b5ec4a87518d



The RRT algorithm samples random points, steers towards them while avoiding collisions, and builds a tree by connecting nodes. The implementation also provides functions for collision checking, path planning, and visualization using Matplotlib. The environment grid is loaded from a file, and users can select different environments. The ROS 2 node subscribes to topics for initial and goal poses, publishes the occupancy grid, and visualizes the path using markers. The project can be executed by running the main function, which initializes the ROS 2 node and starts the RRT planning process.






