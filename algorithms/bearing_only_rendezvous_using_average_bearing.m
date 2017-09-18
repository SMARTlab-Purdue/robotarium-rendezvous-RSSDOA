function [dxi,stop_condition,energy] = bearing_only_rendezvous_using_average_bearing(L,xi)
% This function implements a Bearings-only controller using 
% the average of all bearing measurements as the control inputs
% Author of this code: Ramviyas Parasuraman. ramviyas@purdue.edu

global N desired_distance error_bearing;

dxi = zeros(2, N);
stop_condition = 1;
vmax = 2;
energy = 0; % Lyapunov candidate function

for i = 1:N
    neighbors = topological_neighbors(L, i);
    alpha_ij = zeros(1,length(neighbors)); 
    k =1;
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn; % adding noise to the bearing measurements
        k=k+1;

        p_ij = norm(xi(1:2, i) - xi(1:2, j)) ;
        energy = energy + p_ij;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
    
    % calculate the average of all bearings
    mean_alpha = atan2(mean(sin(alpha_ij)),mean(cos(alpha_ij))); % Calcuating angular mean value

    % For each neighbor, calculate velocities term based on bearings
    if(~isempty(neighbors))
        dxi(1,i) = vmax * cos(mean_alpha) ;
        dxi(2,i) = vmax * sin(mean_alpha) ;
    end
    
end