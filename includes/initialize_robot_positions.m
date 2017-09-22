function r = initialize_robot_positions(r,N,initial_positions)
%This function initializes the robots to the fixed initial positions in the Robotarium environment
%Author: Ramviyas Parasuraman, ramviyas@purdue.edu

%% Using Robotarium utility functions for dynamics transformations and position controller
si_pos_controller = create_si_position_controller('XVelocityGain', 2, 'YVelocityGain', 2);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 2, 'AngularVelocityLimit', pi);

%% Initialization 
threshold = 0.01; % distance threshold (in meters) within which the robots should be positioned
K = length(initial_positions); % Number of assigned position coordinates (input)
% Check if the size of initial positions are same as the number of robots
if(K < N)
    disp('The number of given position coordinates (K) are not same as the total number of robots (N), therefor will move only the first K number of robots as per the inputs');
end
    
%% Moving the robots to the fixed positions
disp('Moving the robots to the given initial position');
for iter=1:1000
    stop_condition = 1; %Stop condition to see if all robots are positioned correctly
    x = r.get_poses();
    dxi = zeros(2, N);
    for i=1:K
        dxi(:, i) =  si_pos_controller(x(1:2, i), initial_positions(:,i));
        if(norm(x(1:2, i) - initial_positions(:,i)) >= threshold)
            stop_condition = 0;
        end
    end
    dxu = si_to_uni_dyn(dxi, x);
    r.set_velocities(1:N, dxu);    
    r.step();
    if (stop_condition == 1) % This condition will be satisfied only if all robots reach their initial positions
        break;
    end
end

%%