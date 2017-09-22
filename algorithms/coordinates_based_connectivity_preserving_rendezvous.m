function [dxi,stop_condition,energy] = coordinates_based_connectivity_preserving_rendezvous(L,xi)
% This function implements a connectivity preserving weighted coordinates based consensus controller 
% following the references below:
% Zavlanos, Michael M., Magnus B. Egerstedt, and George J. Pappas,
% "Graph-theoretic connectivity control of mobile robot networks." 
% Proceedings of the IEEE 99.9 (2011): 1525-1540.
% And
% Ji, Meng, and Magnus Egerstedt,
% "Distributed coordination control of multiagent systems while preserving connectedness." 
% IEEE Transactions on Robotics 23.4 (2007): 693-703.

% Author of this code: Ramviyas Parasuraman. ramviyas@purdue.edu

global N desired_distance error_distance sensing_range;

dxi = zeros(2, N);
stop_condition = 1;
energy = 0;
vmax = 2;

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xvel = zeros(1,length(neighbors));
    yvel = zeros(1,length(neighbors));
    w_ij = zeros(1,length(neighbors));
    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        p_ij = norm(xi(1:2, i) - xi(1:2, j));
        energy = energy + p_ij;
        position_error = error_distance*randn; % simulating an error in the coordinates/distance measurements
        d_ij = p_ij + position_error;

        w_ij(k) = (2*sensing_range^2) /(sensing_range^2 - d_ij.^2)^2; % Calculating weights for each neighbor

        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        xvel(k) = xi(1, j) - xi(1, i) + position_error; % Calculate the velocities based on relative coordinates
        yvel(k) = xi(2, j) - xi(2, i) + position_error; % Calculate the velocities based on relative coordinates
        k=k+1;
        
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
        
    w_ij = w_ij./sum(w_ij); % Normalizing the weight vector to bound the control velocities
    
    % Calculate the linear velocities (for a Single Integrator model)
    if(~isempty(neighbors))
        dxi(1,i) = vmax*sum(w_ij.*xvel) / length(neighbors);
        dxi(2,i) = vmax*sum(w_ij.*yvel) / length(neighbors);
    end
end