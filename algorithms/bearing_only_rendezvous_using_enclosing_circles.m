function [dxi,stop_condition,energy] = bearing_only_rendezvous_using_enclosing_circles(L,xi)

global N sensing_range desired_distance error_bearing ;

dxi = zeros(2, N);
stop_condition = 1;
energy = 0;
vmax=2;

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xpos_j = zeros(1,length(neighbors));
    ypos_j = zeros(1,length(neighbors));
    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn;
        xpos_j(k) = sensing_range*cos(alpha_ij); % assumed x position of robot j with respect to robot i
        ypos_j(k) = sensing_range*sin(alpha_ij); % assumed y position of robot j with respect to robot i
        k = k + 1;
        
        p_ij = norm(xi(1:2, i) - xi(1:2, j))  ;
        energy = energy + p_ij;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
    semicircle = minboundsemicircle(xpos_j,ypos_j); % Calcute the smallest circle that encapsulates all points using the code from "A suite of minimal bounding objects" by John D'Errico (v1.2 23 May 2014) in Mathworks File Exchange.
    center = semicircle.center'; % direction to the target point
    center_norm = norm(center - xi(:,i));  % distance of the center from robot i
    weight = min(sensing_range/2,center_norm); % distance to the target point
    
    if(~isempty(neighbors))
        dxi(:,i) = vmax * weight * center / center_norm; % compute velocity to the target point
    end
end