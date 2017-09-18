# robotarium-rendezvous-RSSDOA

This repository contains the Matlab source codes (to use in Robotarium platform) of various rendezvous controllers for consensus control in a multi-agent / multi-robot system. We also proposed a new weighted bearing controller using wireless network measurements such as the Received Signal Strength (RSS) and the Direction of Arrival (DOA). 

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

# Included utility functions
We have provided the following utility functions to use it in the experiments. They can also be used as a standalone function.
1. GetConnectedGraph: This functions provides a connected graph given the coordinates of the agents and a common sensing range.
2. initialize_robot_positions: This function repositions the randomly positioned robots in the Robotarium simulator to the initial positions supplied as input to the function.
### Third party utility functions
3. minboundsemicircle: This is a third-party function available in Mathworks File Exchange provided by John D'Errico as "A suite of minimal bounding objects" (v1.2). It provides a smallest enclosing circle given a set of points. 
https://www.mathworks.com/matlabcentral/fileexchange/34767-a-suite-of-minimal-bounding-objects?focused=3820668&tab=function

# Contact
For any queries, issues, and questions with the code, please contact the developers Ramviyas Parasuraman (ramviyas@purdue.edu) and Prof.Byung-Cheol Min (minb@purdue.edu).

# Authors
Ramviyas Parasuraamn: http://web.ics.purdue.edu/~rnattanm/ Github: https://github.com/ramviyas

Byung-Cheol Min: http://web.ics.purdue.edu/~minb/

SMART Lab - Purdue University: http://www.smart-laboratory.org/
