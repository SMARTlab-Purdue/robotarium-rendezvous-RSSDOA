function [dxi,stop_condition,energy] = bearing_and_range_based_rendezvous_using_weighted_bearings(L,xi)

global N desired_distance error_bearing error_distance;

dxi = zeros(2, N);
stop_condition = 1;
vmax = 1;
energy = 0;

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xvel = zeros(1,length(neighbors));
    yvel = zeros(1,length(neighbors));
    w_ij = zeros(1,length(neighbors));
    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn;
        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        %dxi(:, i) = dxi(:, i) + (xi(:, j) - xi(:, i));
        p_ij = norm(xi(1:2, i) - xi(1:2, j))  ;

        xvel(k) = cos(alpha_ij) ;
        yvel(k) = sin(alpha_ij) ;
        w_ij(k) = (p_ij + error_distance*randn);
        k = k+1;

        if (p_ij > desired_distance)
            stop_condition = 0;
        end
        energy = energy + p_ij;
    end
    w_ij = w_ij./sum(w_ij); % Normalize the weight vector for a given robot
    if(~isempty(neighbors))
        dxi(1,i) = vmax*sum(w_ij.*xvel) / length(neighbors);
        dxi(2,i) = vmax*sum(w_ij.*yvel) / length(neighbors);
    end
end