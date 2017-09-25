function [dxi,stop_condition,energy] = coordinates_based_rendezvous(L,xi)
% This function implements a linear consensus controller using the
% relative positions information as the control inputs based on the following reference:
% A. Jadbabaie, J. Lin, and A. S. Morse, “Coordination of groups of mobile
% autonomous agents using nearest neighbor rules,” IEEE Trans. Autom. Control, vol. 48, no. 6, pp. 988–1001, Jun. 2003.
% This is also available as an example in Robotarium: https://www.robotarium.gatech.edu/examples/rendezvous
% Author of this code: Ramviyas Parasuraman. ramviyas@purdue.edu

global N desired_distance error_distance;

dxi = zeros(2, N);
stop_condition = 1;
energy = 0;
%vmax = 2;

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xvel = zeros(1,length(neighbors));
    yvel = zeros(1,length(neighbors));
    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        p_ij = norm(xi(1:2, i) - xi(1:2, j));
        energy = energy + p_ij;
        dij_xerror = error_distance*randn; % simulating an error in the coordinates measurements
        dij_yerror = error_distance*randn; % simulating an error in the coordinates measurements

        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        xvel(k) = (xi(1, j) - xi(1, i)) + dij_xerror; % Calculate the velocities based on relative coordinates
        yvel(k) = (xi(2, j) - xi(2, i)) + dij_yerror; % Calculate the velocities based on relative coordinates
        k=k+1;
        
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
    
    % For each neighbor, calculate appropriate consensus term and add it to the total velocity
    if(~isempty(neighbors))
        for k=1:length(neighbors)
            dxi(1, i) = dxi(1, i) + xvel(k); % Calculate the velocities based on relative coordinates
            dxi(2, i) = dxi(2, i) + yvel(k); % Calculate the velocities based on relative coordinates
        end
    end
    % Optional: Averaging the velocities over all neighbors 
    % To use this below option, please uncomment the vmax in line 14
    %dxi(:,i) = vmax*dxi(:,i) / length(neighbors);
end