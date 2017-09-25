function [dxi,stop_condition,energy] = bearing_only_rendezvous_using_enclosing_circles(L,xi)
% This function implements a Bearings-only controller using the smallest enclosing circles 
% of pseudo-positions of the neighbors based on bearing measurements 
% The algorithm is based on the following reference:
% Kriegleder, M., Digumarti, S. T., Oung, R., & D'Andrea, R. (2015, May),
% "Rendezvous with bearing-only information and limited sensing range",
% In 2015 IEEE International Conference on Robotics and Automation (ICRA), (pp. 5941-5947). IEEE.
% Author of this code: Ramviyas Parasuraman. ramviyas@purdue.edu

global N sensing_range desired_distance error_bearing ;

dxi = zeros(2, N);
stop_condition = 1;
energy = 0; % Lyapunov candidate function
vmax=2;
variant = 1; % A flag to choose whether to compute the smallest enclosing circle or semicircle (1 = circle, 2 = semicircle)

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xpos_j = zeros(1,length(neighbors));
    ypos_j = zeros(1,length(neighbors));
    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn; % adding noise to the bearing measurements
        %calculationg pseudo-positions of the neighbors using the sensing_range as the distance between i and j
        xpos_j(k) = sensing_range*cos(alpha_ij); % assumed x position of robot j with respect to robot i
        ypos_j(k) = sensing_range*sin(alpha_ij); % assumed y position of robot j with respect to robot i
        k = k + 1;
        
        p_ij = norm(xi(1:2, i) - xi(1:2, j))  ;
        energy = energy + p_ij;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
    
    if(~isempty(neighbors))
        % calculating the smallest circle or semi-circle contaning all neighbors
        if(variant == 1) % Use smallest circle - results in stable algorithm performance but slower computational efficiency (slower convergence)
            [center,~] = minboundcircle(xpos_j,ypos_j); % Calcute the smallest circle that encapsulates all points using the code from "A suite of minimal bounding objects" by John D'Errico (v1.2 23 May 2014) in Mathworks File Exchange.
            center = center';
        else % Use smallest semicircle - results in faster algorithm convergence (higher computational efficiency) but with little compromise to stability of movements
            semicircle = minboundsemicircle(xpos_j,ypos_j); % Calcute the smallest semicircle that encapsulates all points using the code from "A suite of minimal bounding objects" by John D'Errico (v1.2 23 May 2014) in Mathworks File Exchange.
            center = semicircle.center'; % direction to the target point
        end
        center_norm = norm(center - xi(:,i));  % distance of the center from robot i
        weight = min(sensing_range/2,center_norm); % distance to the target point

        % For each robot, calculate velocities based on enclosing circles of pseudo-positions    if(~isempty(neighbors))
        dxi(:,i) = vmax * weight * (center-xi(:,i)) / center_norm; % compute velocity to the target point
    end
end