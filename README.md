# Multi-Robot Consensus based Rendezvous in Robotarium

This repository contains the Matlab source codes (to use in Robotarium platform) of various rendezvous controllers from the state-of-the-art of consensus control in a multi-agent / multi-robot system. In addition, we propose a consensus controller using wireless netowrk measurements which uses the Received Signal Strength (RSS) measurements and the Direction of Arrival (DOA) of wireless signals estimated using the RSS measurements.

Robotarium is a remotely accessible multi-robot experiment testbed provided by the Georgia Tech. https://www.robotarium.gatech.edu/

# Installation instructions
1. Install the Robotarium simulator available from this repository: https://github.com/robotarium/robotarium-matlab-simulator
2. Download the current repository anywhere on your computer using the command "git clone https://github.com/SMARTlab-Purdue/robotarium-rendezvous-RSSDOA.git"

# Usage instructions
1. First navigate to the Robotarium folder where your downloaded the Robotarium source codes. And Run the "init.m" file to initialize the Matlab workspace and include necessary paths.
2. Navigate to the current repository (robotarium-rendezvous-RSSDOA) folder
3. Add to path (Include) the 'algorithms' and 'includes' directories into your Matlab path.
4. Use the rendezvous_experiments_Robotarium_main.m file and choose your desired rendezvous algorithm (12 options currently available) along with experiment parameters and run this main file.
5. Choose respective consensus control functions file in the "algorithms" folder to change/modify/update the algorithm.

# Included **Algorithm** functions
We have implemented the following consensus control algorithms.

## Newly proposed Weighted Bearings Controller - consensus control using RSS and DOA of wireless signals
* weighted_bearing_consensus_using_RSS_and_DOA: The relative bearings (DOA of wireless signals) of the neighbor robots are used to control the robot's position and direction, whereas the RSS is used to weight each neighbor's impact on the distributed consensus controller. 

## Coordinates based consensus (Rendezvous) algorithms - using relative position measurements
* coordinates_based_rendezvous: (Baseline) It relies on the full coordinates (relative positions) of neighbor robots. This is the standard rendezvous algorithm. See https://www.robotarium.gatech.edu/examples/rendezvous for more information on this algorithm.

* coordinates_based_connectivity_preserving_rendezvous: It is similar to the above but uses weights (artificial potential fields)

* coordinates_based_rendezvous_circumcenter: It relies on the circumcenter of all coordinates (relative positions) of neighbor robots

## State-of-the-art Bearing-only consensus algorithms
* bearing_only_rendezvous_using_all_bearings: It uses bearing measurements of all neighbor robots from each robot

* bearing_only_rendezvous_using_min_and_max_bearings: It uses only the min and max bearings of neighbors from each robot

* bearing_only_rendezvous_using_enclosing_circles: It uses bearing measurmeents and enclosing circles algorithm

## State-of-the-art consensus controllers using both Range and Bearings
* weighted_bearing_consensus_using_Range_and_Bearings: It uses both range and bearing measurements from any sensors as control inputs

## Other (possible) consensus controllers from the state-of-the-art : using both coordinates and bearings based controllers
* bearing_only_rendezvous_using_average_bearing: It uses mean of neighbor bearings from each robot

* coordinates_based_rendezvous_with_mean_velocity: It uses mean of all neighbor directions

* coordinates_based_rendezvous_with_max_velocity: It uses the coordinate of farthest neighbor

* coordinates_based_rendezvous_with_min_velocity: It uses the coordinate of the nearest neighbor

# Included utility functions
We have provided the following utility functions to use it in the experiments. They can also be used as a standalone function.

* GetConnectedGraph: This functions provides a connected graph given the coordinates of the agents and a common sensing range.

* initialize_robot_positions: This function repositions the randomly positioned robots in the Robotarium simulator to the initial positions supplied as input to the function.

### Third party utility functions
* minboundcircle: This is a third-party function that provides a smallest enclosing circle given a set of points. Available in Mathworks File Exchange provided by John D'Errico as "A suite of minimal bounding objects" (v1.2). https://www.mathworks.com/matlabcentral/fileexchange/34767-a-suite-of-minimal-bounding-objects?focused=3820668&tab=function

* minboundsemicircle: This is a third-party function that provide a smallest enclosing semicircle given a set of points. (same reference as the previous function).

# Contact
For any queries, issues, and questions with the code, please contact Ramviyas Parasuraman (ramviyas at purdue dot edu).

# Contributors
Ramviyas Parasuraman (ramviyas at purdue dot edu): http://web.ics.purdue.edu/~rnattanm/ Github: https://github.com/ramviyas

Byung-Cheol Min (minb at purdue dot edu): http://web.ics.purdue.edu/~minb/

SMART Lab - Purdue University: http://www.smart-laboratory.org/
