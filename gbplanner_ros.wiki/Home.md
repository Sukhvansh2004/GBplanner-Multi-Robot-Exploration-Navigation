# Graph-Based Exploration Planner 2.0
Welcome to the gbplanner_ros wiki!

This wiki will guide you through the installation and running of the package along with documentation of the package.

## Planner Overview
GBPlanner 2.0 is an extension of our previous work on Graph-Based Exploration Planner. The new planner presents improved computational performance and better handling of positive and negative obstacles for ground robots. The planner does not require any prior knowledge of the environment other than the general bounds of the volume to be explored. 

The planner uses a bifurcated local and global planning architecture. To maximize the newly mapped volume, the local planner works in a local subspace around the current robot location. The planner builds a 3D graph by sampling vertices in free space and connecting them with admissible edges. For ground robots, the sampled points and the interpolated edges are projected downwards on the map to check ground presence. An edge is admissible for a ground robot if each segment of the interpolated edge is free and within an inclination-limit set according to the robot's locomotion constraints. For aerial robots, an edge is admissible if it lies entirely in the free space. Then shortest paths in the graph are calculated using Dijkstra's algorithm starting from the current robot location and an exploration gain, reflecting the expected amount of volume to be mapped by traversing along each path is calculated. GBPlanner2 provides the choice to perform clustering of vertices for gain calculation and an option to calculate the gain only at the leaf vertices of the shortest paths to speed up the gain calculation process. Certain modifications in the ray casting procedure (used to calculate the gain) are also implemented to improve the computation time further. Finally, the path with the highest exploration gain is selected, improved for safety, and commanded to the robot.  
The global planner is responsible for two tasks 1) repositioning the robot to a previously identified high exploration gain location on the map when the local exploration is exhausted and 2) bringing the robot back to the home location within the specified mission time limit or when the entire volume within the given bounds is explored.

## Acknowledgements
This open-source release is based upon work supported by a) the **Defense Advanced Research Projects Agency (DARPA)** under Agreement No. HR00111820045, b) the **Research Council of Norway project SENTIENT** (Project No. 321435), and c) the **USDA NIFA Award** 2020-67021-30754

## Project contributing to the development of the planner
- **DARPA Subterranean Challenge - The project in which the method was developed initially**
- SENTIENT (Funding by Research Council of Norway)
- DIGIFOREST (Funding through Horizon Europe)

## Contents
For specific instructions please visit the respective pages:
1. [Installation](Installation)
2. [Demo Simulation](Demo)
3. [Parameters](Parameters)
4. [Interface](Interface)
5. [Experimental results](Experiments)