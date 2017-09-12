function [dxi,stop_condition,energy] = bearing_only_rendezvous_using_all_bearings(L,xi)

global N desired_distance error_bearing;

dxi = zeros(2, N);
stop_condition = 1;
vmax = 1;

energy = 0;
for i = 1:N
    neighbors = topological_neighbors(L, i);
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn;
        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        %dxi(:, i) = dxi(:, i) + (xi(:, j) - xi(:, i));
        dxi(1,i) = dxi(1,i) + cos(alpha_ij) ;
        dxi(2,i) = dxi(2,i) + sin(alpha_ij) ;
        
        p_ij = norm(xi(1:2, i) - xi(1:2, j)) ;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
        energy = energy + p_ij;
    end
    if(~isempty(neighbors))
        dxi(1,i) = vmax*dxi(1,i) / length(neighbors);
        dxi(2,i) = vmax*dxi(2,i) / length(neighbors);
    end
end