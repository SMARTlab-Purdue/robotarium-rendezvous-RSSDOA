%% This is the main file to conduct experiments with several consensus control (rendezvous) algorithms of N robots in the Robotarium testbed
%Ramviyas Parasuraman, ramviyas@purdue.edu. 
%clear all;
close all;

global sensing_range error_bearing error_distance uni_to_si_states si_to_uni_dyn G N desired_distance;

%% Number of robots
N=15; % Number of agents/robots

%% Choose your Rendezvous algorithm
% Newly Proposed Weighted Bearing Controllers
algorithm = 'weighted_bearing_consensus_using_RSS_and_DOA'; % It uses the DOA of RSS and the RSS form wireless nework measurements as control inputs
%algorithm = 'weighted_bearing_consensus_using_Range_and_Bearings'; % It uses both range and bearing measurements from any sensors as control inputs

% Baseline Algorithms - Coordinates based consensus(Rendezvous) algorithms
%algorithm = 'coordinates_based_rendezvous' ; % It relies on the full coordinates (relative positions) of neighbor robots
%algorithm = 'coordinates_based_connectivity_preserving_rendezvous' ; % It is similar to the above but uses weights (artificial potential fields)
%algorithm = 'coordinates_based_rendezvous_circumcenter' ; % It relies on the circumcenter of all coordinates (relative positions) of neighbor robots


% State of the Art (SOTA) Bearing-only consensus(Rendezvous) algorithms
%algorithm = 'bearing_only_rendezvous_using_all_bearings';
%algorithm = 'bearing_only_rendezvous_using_min_and_max_bearings';
%algorithm = 'bearing_only_rendezvous_using_enclosing_circles';

% Other possible consensus controllers 
%algorithm = 'bearing_only_rendezvous_using_average_bearing';
%algorithm = 'coordinates_based_rendezvous_with_mean_velocity';
%algorithm = 'coordinates_based_rendezvous_with_max_velocity';
%algorithm = 'coordinates_based_rendezvous_with_min_velocity';

fH = str2func(algorithm); % function handle for the chosen rendezvous algorithm

%% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();
% Build the Robotarium simulator object!
r = rb.set_number_of_agents(N).set_save_data(false).build();
figure_robotarium = figure(1); movegui('northeast'); movegui('onscreen');
%title('Rendezvous algorithm experimented in Robotarium testbed');

%% Experiment parameters
desired_distance = 0.1; % desired inter-agent distance range to realize stop condition
sensing_range = 0.8; % Sensing radius within which robot i detects robot j (same for all the robots)
error_bearing = 0.1; % Standard deviations of the bearing measurment error (radians)
error_distance = 0.1; % Standard deviations of the distance measurment error (m)
root_node = 5; % Set the root node robot where the rendezvous should occur (instead of a consensus point), if rendezvous_at_root flag is set/used below
desired_energy = 0.2; % (currently not used) desired value of the Lyapunov candidate energy function
safety_radius = 0.11; % Do NOT change this - set for Robotarium. It is the safety radius for collision avoidance between robots
dxmax = 0.075; % Do NOT change this - set for Robotarium. The maximum velocity of the GRITSbots robots (<=0.8m/s) (used if normalize_velocities is set/used below)

%% Flags to use specific parts of the code
collision_avoidance = 0; % To enable/disable barrier certificates
normalize_velocities = 1; % To normalize the velocities (recommended - Do NOT reset)
initial_positions_fixed = 1; % If set, then fixed initial positions (defined further down) will be used instead of random
update_network_topology = 1; % To enable/disable the update of connected graph (dynamically) in every iteration
rendezvous_at_root = 0; % If set, it will make all the robots rendezvou at a root node (defined below) instead of a common location
formation_control = 0; % If set, the robots will create a specified formation (shape) instead of rendezvous
% Flags for Plotting tools
plot_initial_graph = 0; % To plot initial connected graph
plot_dynamic_graph = 0; % To plot updated connected graph in every iteration
plot_robot_index = 1; % To enable/disable the display of robot index on top of each robot in the Robotarium figure
plot_robot_trajectory = 0; % To enable/disable the display of robot trajectory in the Robotarium figure
plot_robot_initialposition = 0; % To enable/disable the display of robot initial position in the Robotarium figure
make_movie = 0; % To create a movie using the Robotarium figure over the iterations

%% Formation control parameters (if formation_control flag is set)
% to replicate a P-like formation
b = floor(N/2);
c = N-floor(N/2);
angle = 0:(360/c):359; 
target_x = 0.5*cos(pi)*ones(1,b) ;
target_y = 0:-1.4/(b-1):-1.4;
target_configuration = [[target_x,0.5*cos(angle*pi/180)];[target_y,0.5*sin(angle*pi/180)]];
% to replicate a circular formation
%angle = 0:(360/N):359; 
%target_configuration = [0.5*cos(angle*pi/180);0.5*sin(angle*pi/180)];


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

%% Initialize the robots to fixed positions (if initial_positions_fixed is set)
if(initial_positions_fixed == 1)
    initial_positions(1,:) = r.poses(1,:) *2; % For random initial positions with wider distribution/coverage
    initial_positions(2,:) = r.poses(2,:) *2; % For random initial positions with wider distribution/coverage
    %initial_positions = [-0.2 -0.5 0.5 0.4 -0.1; 0.2 -0.4 -0.1 0.6 0.7]; % for N=5
    %initial_positions = [0 0.4 0.5 0.4 -0.1 -0.3 -0.5 -0.7 0 1 -1 -1 0.3 -0.5 0.9; 0.3 0.9 1.1 -1 -0.2 -0.9 -0.3 -1 1.2 -1.2 0.2 -0.9 -0.4 0.6 1]; % N=15
    %initial_positions = [0 0.4 0.5 0.6 -0.1 -0.3 -0.5 -0.7 0 0.8 -1.1 -1 0.3 -0.5 0.9 0.5 -0.5 -1 1 1.4; 0.3 0.9 1.1 -1 -0.2 -0.9 -0.3 -1 1.2 -1.2 0.2 -0.9 -0.4 0.6 1 0.5 1.3 1 -0.2 0.2]; % for N=20
    r = initialize_robot_positions(r,N,initial_positions);

end

%% Finding the connected graph (Also checking if the initial interation/network graph is connected)
x = r.get_poses();
xi = uni_to_si_states(x);
r.step();
[L,G] = GetConnectedGraph(x(1:2,:),sensing_range); % Getting the initial connected Graph
if(length(find(eig(L) <=0.001)) > 1)
    connection_status_initial = 0; % disconnected graph
    disp('Initial graph has some disconnected components. For experiments, we need a connected graph. Please try again!');
    return; % If it's a disconnected graph, then stop the script
end

%% Initiating connected graph figure window
if(plot_initial_graph == 1)
    figure_graph = figure(2); plot(G); title('Initial Network/Interaction Topology');
    xlim([-3 3]);
    ylim([-3 3]);
    movegui('northwest');
end

%% Experiment initialization
max_iterations = 1000; % the number of iterations for the experiment

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);

previous_xi = xi; % A temporary variable to store the position values
distance_travelled = zeros(1,N); % total distance traveled by each robot - Performance evaluation metric
iteration_at_stopcondition = 0; % number of iteration at which the stop condition is reached
iteration_at_minenergy = 0; % number of iteration at which the energy function values is the minimum (less than a threshold)
energy = zeros(1,max_iterations); % The value of the Energy function which is sum of all distances between the connected nodes
mycols = jet(N); % To display colored trajectory for each robot (if plot_robot_trajectory is set)
fig_index = figure_robotarium;
fig_traj = figure_robotarium;
Movieframe(1:max_iterations) = gcf; % to be used in movie creation if make_movie flag is set/used

%% Display the robot's initial position trajectory in the Robotarium figure
if(plot_robot_initialposition == 1)  
    %fig_traj = set(0,'CurrentFigure',r.figure_handle);
    for i=1:N
        fig_traj = plot(x(1,i),x(2,i),'o','Color',mycols(i,:));
    end
end
    
disp('Rendezvous process initiated - displaying the number of iterations');

%% Display the title text in the Robotarium figure
set(0,'CurrentFigure',r.figure_handle);
alg_string = regexprep(algorithm,'_',' ');
alg_string = regexprep(alg_string,'\s*.','${upper($0)}');
%alg_string = strcat('Algorithm: ',alg_string);
text(-1.4,1.4,alg_string,'FontSize',8,'Color','red','FontWeight','Bold', 'Interpreter', 'none');

%% Iteration starts here (for the previously specified number of iterations)
for t = 1:max_iterations
    disp(t) % to display the iteration number
    %stop_condition = 1; % This variable is to define the stop condition. If it's 1 - then stop the iteration/experiment

    set(0,'CurrentFigure',r.figure_handle);
    fig_iter = text(-1.4,-1.4,strcat('Iteration :',' ',num2str(t)),'FontSize',10,'Color','red','FontWeight','Bold');

    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds in Robotarium
    x = r.get_poses(); % Get unicycle coordinates (x,y,theta)
    xi = uni_to_si_states(x); % convert the unicycle pose to SI units (x,y)
    
    %% Formation control (if formation_control is set/used)
    if(formation_control == 1)
        xi = xi - target_configuration;
    end
    
    %% Display the robot's index on top of each robot in the Robotarium
    if(plot_robot_index == 1)  
        for i=1:N
            set(0,'CurrentFigure',r.figure_handle);
            fig_index(i) = text(x(1,i),x(2,i)+0.04,num2str(i),'FontSize',6,'Color','red','FontWeight','Bold');
        end
    end

    %% Display the robot's trajectory in the Robotarium figure
    if(plot_robot_trajectory == 1)  
        for i=1:N
            set(0,'CurrentFigure',r.figure_handle);
            fig_traj(i) = plot(x(1,i),x(2,i),'.--','Color',mycols(i,:));
        end
    end
    
    %% Update the connected tree dynamically
    if (update_network_topology == 1)
        [L,G] = GetConnectedGraph(x(1:2,:),sensing_range); % Finding the initial connected Graph
    end
    
    %% Consensus/Rendezvous Algorithm 
    [dxi,stop_condition,energy(t)] = fH(L,xi); % The rendezvous algorithm chosen in the beginning
    
    %% Change the Rendezvous location (if need to rendezvou at a specific root node)
    if(rendezvous_at_root == 1 && formation_control ~= 1) % It can work only for rendezvous and not for formation control
        dxi(1,:) = dxi(1,:) - dxi(1,root_node);
        dxi(2,:) = dxi(2,:) - dxi(2,root_node);
    end
    
    %% Plotting the connected graph
    if(plot_dynamic_graph == 1)
        set(0,'CurrentFigure',figure_graph);   
        plot(G); title('Dynamic Network Topology'); 
        xlim([-3 3]);
        ylim([-3 3]);
    end
        
    %% Normalizing the velocity limits
    if(normalize_velocities == 1)
        for i = 1:N
            if norm(dxi(:,i)) > dxmax
                dxi(:,i) = dxi(:,i)/norm(dxi(:,i))*dxmax;
            end
        end
    end
    %% Utilize barrier certificates for inter-robot collision avoidance
    if(collision_avoidance == 1)
        dxi = si_barrier_cert(dxi, x);
    end
    
    %% Transform the single-integrator to unicycle dynamics using the transformation we created earlier
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send/Set velocities to agents
    r.set_velocities(1:N, dxu);
    r.step(); % This function must be called to properly set the velocities and execute them
      
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
    
    %% Movie making
    if(make_movie == 1)
        set(0,'CurrentFigure',r.figure_handle);
        Movieframe(t) = getframe(gcf);
    end
    
    %% Deleting unncessary figure/text handles
    if(plot_robot_index == 1)  
        delete(fig_index); % delete the text objects (robot indices) on the Robotarium figure
    end
    delete(fig_iter);
    
    
end

% Closing the Robotarium object properly
r.call_at_scripts_end();

%% Movie making
if(make_movie == 1)
    v = VideoWriter('robotarium_simulation_expoeriment.mp4','MPEG-4');
    open(v);
    writeVideo(v,Movieframe);
    close(v);
end

%% Saving the final figure if needed
%print(figure_robotarium,'RobotariumFigure','-depsc');