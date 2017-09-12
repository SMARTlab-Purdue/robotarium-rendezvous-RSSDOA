function [dxi,stop_condition,energy] = coordinates_based_rendezvous_with_max_velocity(L,xi)

global N desired_distance error_distance;

dxi = zeros(2, N);
stop_condition = 1;
energy = 0;
%vmax = 2;

for i = 1:N
    neighbors = topological_neighbors(L, i);
    % Iterate through agent i's neighbors
    xvel = zeros(1,length(neighbors));
    yvel = zeros(1,length(neighbors));
    k=1;
    for j = neighbors
        p_ij = norm(xi(1:2, i) - xi(1:2, j));
        energy = energy + p_ij;
        
        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        xvel(k) = (xi(1, j) - xi(1, i)) + error_distance*randn; % Calculate the velocities based on relative coordinates
        yvel(k) = (xi(2, j) - xi(2, i)) + error_distance*randn; % Calculate the velocities based on relative coordinates
        k=k+1;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
    
    %Averaging the velocities over all neighbors
    if(~isempty(neighbors))
        dxi(1,i) = max(xvel) ;
        dxi(2,i) = max(yvel) ;
    end

end