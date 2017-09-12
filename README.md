# robotarium-rendezvous-RSSDOA

This repository contains the Matlab source codes (to use in Robotarium platform) of various rendeavous controllers for consensus control in a multi-agent or multi-robot system, including the proposed weighted controller using wireless network measuremnts such as the Received Signal Strength (RSS) and the Direction of Arrival (DOA). 

Robotarium is a remotely accessible multi-robot experiment testbed provided by the Georgia Tech. https://www.robotarium.gatech.edu/

# Installation instructions
1. Install the Robotarium simulator available from this repository: https://github.com/robotarium/robotarium-matlab-simulator
2. Download the current repository anywhere on your computer using the command "git clone https://github.com/SMARTlab-Purdue/robotarium-rendezvous-RSSDOA.git"
# Usage instructions
1. First navigate to the Robotarium folder where your downloaded the Robotarium source codes. And Run the "init.m" file to initialize the Matlab workspace and include necessary paths.
2. Navigate to the current repository (robotarium-rendezvous-RSSDOA) folder
3. Use the rendezvous_experiments_Robotarium_main.m file and choose your desired rendezvous algorithm (8 options currently available) along with experiment parameters and run this main file.
4. Choose respective consensus control functions file to change/modify/update the algorithm.

# Included utility functions
We have provided some utility functions to use it in the experiments. They are as follows.
1. GetConnectedGraph: This functions provides a connected graph given the coordinates of the agents and a common sensing range.
2. initialize_robot_positions: This function repositions the randomly positioned robots in the Robotarium simulator to a defined initial positions. 

# Third party utility functions
2. minboundsemicircle: This function provides a smallest circle given a set of points. It is available in Mathworks File Exchange provided by John D'Errico as "A suite of minimal bounding objects" (v1.2). 
https://www.mathworks.com/matlabcentral/fileexchange/34767-a-suite-of-minimal-bounding-objects?focused=3820668&tab=function

# Contact
For any queries, issues, and questions with the code, please contact Ramviyas Parasuraman (ramviyas@purdue.edu).
