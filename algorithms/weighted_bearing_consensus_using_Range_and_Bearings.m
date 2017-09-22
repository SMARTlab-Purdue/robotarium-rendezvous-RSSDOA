function [dxi,stop_condition,energy] = weighted_bearing_consensus_using_Range_and_Bearings(L,xi)
% This function implements the Weighted Bearings Consensus Controller 
% using Bearings and Range information measured using any sensors as the (noisy) control inputs.
% The weight function is inspired by the following reference (Equation 24):
% Zavlanos, Michael M., Magnus B. Egerstedt, and George J. Pappas,
% "Graph-theoretic connectivity control of mobile robot networks.",
% Proceedings of the IEEE 99.9 (2011): 1525-1540.
% Author: Ramviyas Parasuraman. ramviyas@purdue.edu

global N desired_distance error_bearing error_distance sensing_range;

dxi = zeros(2, N);
stop_condition = 1;
vmax = 2; % Max linear velocity
energy = 0; % Lyapunov candidate function

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xvel = zeros(1,length(neighbors));
    yvel = zeros(1,length(neighbors));
    w_ij = zeros(1,length(neighbors));
    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn; % adding noise to the bearing measurements
        p_ij = norm(xi(1:2, i) - xi(1:2, j));
        d_ij = p_ij + error_distance*randn ; % adding noise to the distance (range) measurements

        xvel(k) = cos(alpha_ij) ;
        yvel(k) = sin(alpha_ij) ;
             
        w_ij(k) = d_ij.^2 /(sensing_range^2 - d_ij.^2); % Calculating weights for each neighbor
        k = k+1;

        if (p_ij > desired_distance)
            stop_condition = 0;
        end
        energy = energy + p_ij;
    end
    
    w_ij = w_ij./sum(w_ij); % Normalizing the weight vector to bound the control velocities
    
    % Calculate the linear velocities (for a Single Integrator model)
    if(~isempty(neighbors))
        dxi(1,i) = vmax*sum(w_ij.*xvel) / length(neighbors);
        dxi(2,i) = vmax*sum(w_ij.*yvel) / length(neighbors);
    end
end