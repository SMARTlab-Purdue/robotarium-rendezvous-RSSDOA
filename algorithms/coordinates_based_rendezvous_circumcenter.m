function [dxi,stop_condition,energy] = coordinates_based_rendezvous_circumcenter(L,xi)
% This function implements a consensus controller using the
% relative positions information as the control inputs based on the following reference:
% H. Ando, Y. Oasa, I. Suzuki and M. Yamashita, 
% "Distributed memoryless point convergence algorithm for mobile robots with limited visibility," 
% in IEEE Transactions on Robotics and Automation, vol. 15, no. 5, pp. 818-828, Oct 1999.
% doi: 10.1109/70.795787
% In future, this algorihm will eventually be revised to the improved CircumCenter Algorithm (Cortes et al. 2006)
% "Robust rendezvous for mobile autonomous agents via proximity graphs in arbitrary dimensions."
% Author of this code: Ramviyas Parasuraman. ramviyas@purdue.edu

global N desired_distance error_distance sensing_range;

dxi = zeros(2, N);
stop_condition = 1;
energy = 0;
vmax = 2;
dmax = 0.1; % the assumed maximum distance a robot can travel in one iteration

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xpos_j = zeros(1,length(neighbors));
    ypos_j = zeros(1,length(neighbors));
    theta_j = zeros(1,length(neighbors));

    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        p_ij = norm(xi(1:2, i) - xi(1:2, j))  ;
        energy = energy + p_ij;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
        
        dij_xerror = error_distance*randn; % simulating an error in the coordinates measurements
        dij_yerror = error_distance*randn; % simulating an error in the coordinates measurements
        xpos_j(k) = xi(1,j) + dij_xerror ; % assumed x position of robot j with respect to robot i
        ypos_j(k) = xi(2,j) + dij_yerror ; % assumed y position of robot j with respect to robot i
        theta_j(k) = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i));
        k = k + 1;        
    end
    
    if(~isempty(neighbors))
        % calculating the smallest circle contaning all neighbors
        [center,~] = minboundcircle(xpos_j,ypos_j); % Calcute the smallest circle that encapsulates all points using the code from "A suite of minimal bounding objects" by John D'Errico (v1.2 23 May 2014) in Mathworks File Exchange.
        center = center';
        center_norm = norm(center - xi(:,i));  % distance of the center from robot i
        theta_jc = abs(theta_j - atan2(center(2)-xi(2,i),center(1)-xi(1,i))); % angle between the rays pi-pi (i to j), and pi-ci (i to center)
        limit_j = (p_ij/2).*cos(theta_jc) + sqrt((sensing_range/2).^2 - (p_ij*sin(theta_jc)/2).^2); % calculate the distance limit for each neighbor
        weight = min([min(limit_j),center_norm,dmax]); % calculate the distance to the target point
        % For each robot, calculate velocities based on enclosing circles of pseudo-positions    if(~isempty(neighbors))
        dxi(:,i) = vmax * weight * (center - xi(:,i)) / center_norm; % compute velocity to the target point
    end
end