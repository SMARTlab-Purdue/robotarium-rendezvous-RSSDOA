%% This is the main file to conduct experiments for several consensus control (rendezvous) algorithms of N robots in the Robotarium testbed
%Ramviyas Parasuraman, ramviyas@purdue.edu. 
close all;

global sensing_range error_bearing error_distance uni_to_si_states si_to_uni_dyn si_pos_controller G N desired_distance;

%% Choose your Rendezvous algorithm - For options see the code block between lines 108 and 120
algorithm = 'consensus_control_using_RSS_and_DOA';
fH = str2func(algorithm); % function handle for the chosen rendezvous algorithm

%% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();
N=15; % Number of agents/robots
% Build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();
figure(1); movegui('northeast'); movegui('onscreen');
%title('Rendezvous algorithm experimented in Robotarium testbed');

%% Experiment parameters
desired_distance = 0.1; % desired inter-agent distance range to realize stop condition
desired_energy = 0.2; % desired value of the Lyapunov candidate energy function 
sensing_range = 0.8; % Sensing radius within which robot i detects robot j (same for all the robots)
safety_radius = 0.04; % safety radius for collision avoidance between robots
dxmax = 1; % if normalize_velocities is used
error_bearing = 0.0; % Standard deviations of the bearing measurment error (radians)
error_distance = 0.05; % Standard deviations of the distance measurment error (m)

%% Flags to use specific parts of the code
collision_avoidance = 0; % To enable/disable barrier certificates
normalize_velocities = 1; % To normalize the velocities (recommended)
update_network_topology = 1; % To enable/disable the update of connected graph (dynamically) in every iteration
plot_initial_graph = 0; % To plot initial connected graph
plot_dynamic_graph = 0; % To plot updated connected graph in every iteration
plot_robot_index = 1; % To enable/disable the display of robot index on top of each robot

%% Grab tools we need to convert from single-integrator to unicycle dynamics
%Gains for the transformation from single-integrator to unicycle dynamics
linearVelocityGain = 2; %1
angularVelocityGain = pi;
transformation_gain = 0.06;

% Gain for the diffeomorphism transformation between single-integrator and
% unicycle dynamics
[~, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', transformation_gain);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', linearVelocityGain, 'AngularVelocityLimit', angularVelocityGain);
% Single-integrator position controller
si_pos_controller = create_si_position_controller('XVelocityGain', 2, 'YVelocityGain', 2);
% Collision avoidance - barrier certificates
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', safety_radius);

%% Initialize the robots to a fixed position
initial_positions = [0 0.4 0.5 0.4 -0.1 -0.3 -0.5 -0.7 0 1 -1 -1 0.3 -0.5 0.9; 0.3 0.9 1.1 -1 -0.2 -0.9 -0.3 -1 1.2 -1.2 0.2 -0.9 -0.4 0.6 1];
%initial_positions = r.poses(1:2,:) *2; % For random initial positions
r = initialize_robot_positions(r,initial_positions);


%% Finding the connected tree based on initial positions of the robots
x = r.get_poses();
xi = uni_to_si_states(x);
r.step();
[L,G] = GetConnectedGraph(x(1:2,:),sensing_range); % Finding the initial connected Graph

%% Initiating connected graph figure window

if(plot_initial_graph == 1)
    figure(2); plot(G); title('Initial Network Topology');
    movegui('northwest');
end

%% Experiments initialize
max_iterations = 1000; % the number of iterations for the experiment

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);

previous_xi = xi; % A temporary variable to store the position values
distance_travelled = zeros(1,N); % total distance traveled by each robot - Performance evaluation metric
iteration_at_stopcondition = 0; % number of iteration at which the stop condition is reached
iteration_at_minenergy = 0; % number of iteration at which the energy function values is the minimum (less than a threshold)

energy = zeros(1,max_iterations); % The value of the Energy function which is sum of all distances between the connected nodes


disp('Rendezvous process initiated - displaying the number of iterations');

%Iteration starts here (for the previously specified number of iterations)
for t = 1:max_iterations
    disp(t) % to display the iteration number
    %stop_condition = 1; % This variable is to define the stop condition. If it's 1 - then stop the iteration/experiment 
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds in Robotarium
    x = r.get_poses(); % Get unicycle coordinates (x,y,theta)
    xi = uni_to_si_states(x); % convert the unicycle pose to SI units (x,y)
    
    % Display the robot's index on top of each robot in the Robotarium
    if(plot_robot_index == 1)  
        fig = set(0,'CurrentFigure',r.figure_handle);
        for i=1:N
            fig(i) = text(x(1,i),x(2,i)+0.05,num2str(i),'FontSize',12,'Color','red','FontWeight','Bold');
        end
    end
    
    % Update the connected tree dynamically
    if (update_network_topology == 1)
        [L,G] = GetConnectedGraph(x(1:2,:),sensing_range); % Finding the initial connected Graph
    end
    
    %% Rendezvous Algorithm
    %%% Coordinates based consensus(Rendezvous) algorithms
    %[dxi,stop_condition,energy(t)] = coordinates_based_rendezvous(L,xi); % This is the baseline algorithm
    %[dxi,stop_condition,energy(t)] = coordinates_based_rendezvous_with_max_velocity(L,xi);
    %%% Bearing-only consensus(Rendezvous) algorithms
    %[dxi,stop_condition,energy(t)] = bearing_only_rendezvous_using_all_bearings(L,xi);
    %[dxi,stop_condition,energy(t)] = bearing_only_rendezvous_using_min_and_max_bearings(L,xi);
    %[dxi,stop_condition,energy(t)] = bearing_only_rendezvous_using_average_bearing(L,xi);
    %[dxi,stop_condition,energy(t)] = bearing_only_rendezvous_using_enclosing_circles(L,xi);
    %%% Both Bearing and Range based consensus(Rendezvous) algorithms
    %[dxi,stop_condition,energy(t)] = bearing_and_range_based_rendezvous_using_weighted_bearings(L,xi);
    %%% Consensus(Rendezvous) using Weighted Bearing approach based on wireless Received Signal Strength (RSS) and Direction of Arrival (DOA) estimation
    %[dxi,stop_condition,energy(t)] = consensus_control_using_RSS_and_DOA(L,xi); %
    [dxi,stop_condition,energy(t)] = fH(L,xi);
    
    %% Plotting the connected graph
    if(plot_dynamic_graph == 1)
        figure(2); plot(G); title('Dynamic Network Topology'); 
    end
        
    %% Normalizing the velocity limits
    if(normalize_velocities == 1)
        for i = 1:N
            if norm(dxi(:,i)) > dxmax
                dxi(:,i) = dxi(:,i)/norm(dxi(:,i))*dxmax;
            end
        end
    end
    %% Utilize barrier certificates
    
    if(collision_avoidance == 1)
        dxi = si_barrier_cert(dxi, x);
    end
    
    % Transform the single-integrator to unicycle dynamics using the transformation we created earlier
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!    
    r.step();
    
    %% Performance evaluation metrics
    % Calculate the distance travelled in each iteration
    for i=1:N
        distance_travelled(i) = distance_travelled(i) + norm(xi(:,i)-previous_xi(:,i));
    end
    previous_xi = xi;
    
    %if (stop_condition == 1 && length(leaves) == N-1)
    if ((stop_condition == 1) && (iteration_at_stopcondition == 0))
        display('Stop condition (all inter-robot distances within 0.1m) reached ');
        iteration_at_stopcondition = t;
    end
    
    if((energy(t) <= desired_energy) && (iteration_at_minenergy == 0))
        display('Minimum energy condition (E<=0.2) reached');
        iteration_at_minenergy = t;
    end
    
    if(plot_robot_index == 1)  
        delete(fig); % delete the text objects (robot indices) on the Robotarium figure
    end
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

%save('rssdoa_eb0p35_ed0p05_dd0p1_de0p2.mat','distance_travelled','energy','iteration_at_stopcondition','iteration_at_minenergy');