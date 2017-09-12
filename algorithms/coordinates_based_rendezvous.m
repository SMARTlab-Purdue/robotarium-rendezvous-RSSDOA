function [dxi,stop_condition,energy] = coordinates_based_rendezvous(L,xi)

global N desired_distance error_distance;

dxi = zeros(2, N);
stop_condition = 1;
energy = 0;
%vmax = 2;

for i = 1:N
    neighbors = topological_neighbors(L, i);
    % Iterate through agent i's neighbors
    for j = neighbors
        p_ij = norm(xi(1:2, i) - xi(1:2, j));
        energy = energy + p_ij;
        
        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        dxi(1, i) = dxi(1, i) + (xi(1, j) - xi(1, i)) + error_distance*randn; % Calculate the velocities based on relative coordinates
        dxi(2, i) = dxi(2, i) + (xi(2, j) - xi(2, i)) + error_distance*randn; % Calculate the velocities based on relative coordinates

        
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
    
    %Averaging the velocities over all neighbors
    %dxi(:,i) = vmax*dxi(:,i) / length(neighbors);
end