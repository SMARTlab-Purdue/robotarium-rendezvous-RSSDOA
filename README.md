# Multi-Robot Consensus based Rendezvous in Robotarium

This repository contains the Matlab source codes (to use in Robotarium platform) of various rendezvous controllers for consensus control in a multi-agent / multi-robot system. We also proposed a new weighted bearing controller using wireless network measurements such as the Received Signal Strength (RSS) and the Direction of Arrival (DOA) of the RSS. 

Robotarium is a remotely accessible multi-robot experiment testbed provided by the Georgia Tech. https://www.robotarium.gatech.edu/

# Installation instructions
1. Install the Robotarium simulator available from this repository: https://github.com/robotarium/robotarium-matlab-simulator
2. Download the current repository anywhere on your computer using the command "git clone https://github.com/SMARTlab-Purdue/robotarium-rendezvous-RSSDOA.git"

# Usage instructions
1. First navigate to the Robotarium folder where your downloaded the Robotarium source codes. And Run the "init.m" file to initialize the Matlab workspace and include necessary paths.
2. Navigate to the current repository (robotarium-rendezvous-RSSDOA) folder
3. Add to path (Include) the 'algorithms' and 'includes' directories into your Matlab path.
4. Use the rendezvous_experiments_Robotarium_main.m file and choose your desired rendezvous algorithm (8 options currently available) along with experiment parameters and run this main file.
5. Choose respective consensus control functions file in the "algorithms" folder to change/modify/update the algorithm.

# Included **Algorithm** functions
We have implemented the following consensus algorithms.
## Newly Proposed Weighted Bearing Controllers
1. weighted_bearing_consensus_using_RSS_and_DOA: It uses the DOA of RSS and the RSS form wireless nework measurements as control inputs
2. weighted_bearing_consensus_using_Range_and_Bearings: It uses both range and bearing measurements from any sensors as control inputs

## Coordinates based consensus (Rendezvous) algorithms - using relative position measurements
3. coordinates_based_rendezvous: (Baseline) It relies on the full coordinates (relative positions) of neighbor robots
4. coordinates_based_connectivity_preserving_rendezvous: It is similar to the above but uses weights (artificial potential fields)
5. coordinates_based_rendezvous_circumcenter: It relies on the circumcenter of all coordinates (relative positions) of neighbor robots

## State of the Art (SOTA) Bearing-only consensus(Rendezvous) algorithms
6. bearing_only_rendezvous_using_all_bearings: It uses bearing measurements of all neighbor robots from each robot
7. bearing_only_rendezvous_using_min_and_max_bearings: It uses only the min and max bearings of neighbors from each robot
8. bearing_only_rendezvous_using_enclosing_circles: It uses bearing measurmeents and enclosing circles algorithm

## Other (possible) consensus controllers - both coordinates and bearings based controllers
9. bearing_only_rendezvous_using_average_bearing: It uses mean of neighbor bearings from each robot
10. coordinates_based_rendezvous_with_mean_velocity: It uses mean of all neighbor directions
11. coordinates_based_rendezvous_with_max_velocity: It uses the coordinate of farthest neighbor
12. coordinates_based_rendezvous_with_min_velocity: It uses the coordinate of the nearest neighbor

# Included utility functions
We have provided the following utility functions to use it in the experiments. They can also be used as a standalone function.
1. GetConnectedGraph: This functions provides a connected graph given the coordinates of the agents and a common sensing range.
2. initialize_robot_positions: This function repositions the randomly positioned robots in the Robotarium simulator to the initial positions supplied as input to the function.

### Third party utility functions
3. minboundcircle: This is a third-party function that provides a smallest enclosing circle given a set of points. Available in Mathworks File Exchange provided by John D'Errico as "A suite of minimal bounding objects" (v1.2). https://www.mathworks.com/matlabcentral/fileexchange/34767-a-suite-of-minimal-bounding-objects?focused=3820668&tab=function
4. minboundsemicircle: This is a third-party function that provide a smallest enclosing semicircle given a set of points. (same reference as the previous function).

# Video demonstration
For more information on the proposed and the state-of-the-art controllers, please visit this video demonstration in YouTube: https://youtu.be/PzFVuA4wIP8.

# Contact
For any queries, issues, and questions with the code, please contact the principal developer Ramviyas Parasuraman (ramviyas at purdue dot edu).

# Authors
Ramviyas Parasuraman (ramviyas at purdue dot edu): http://web.ics.purdue.edu/~rnattanm/ Github: https://github.com/ramviyas

Byung-Cheol Min (minb at purdue dot edu): http://web.ics.purdue.edu/~minb/

SMART Lab - Purdue University: http://www.smart-laboratory.org/
