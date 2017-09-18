function [dxi,stop_condition,energy] = bearing_only_rendezvous_using_min_and_max_bearings(L,xi)
% This function implements a linear Bearings-only controller using only the
% minimum and maximum bearings (to represent an angular sector) as the control inputs based on the following reference:
% Zheng, R., & Sun, D. (2013). Rendezvous of unicycles: A bearings-only and perimeter shortening approach. Systems & Control Letters, 62(5), 401-407.
% autonomous agents using nearest neighbor rules,” IEEE Trans. Autom. Control, vol. 48, no. 6, pp. 988–1001, Jun. 2003.
% Author of this code: Ramviyas Parasuraman. ramviyas@purdue.edu
%Note: This algorithm requires that the initial graph is fully (completely)
%connected. However, it may work even with a non-fully connected graph.

global N desired_distance error_bearing;

dxi = zeros(2, N);
stop_condition = 1;
vmax = 2;
energy = 0; % Lyapunov candidate function

for i = 1:N
    neighbors = topological_neighbors(L, i);
    alpha_ij = zeros(1,length(neighbors)); 
    k =1;
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij(k) = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn; % adding noise to the bearing measurements
        k=k+1;
        p_ij = norm(xi(1:2, i) - xi(1:2, j)) ;
        energy = energy + p_ij;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
    end
    
    % For each robot, calculate velocities based on min and max bearings
    if(~isempty(neighbors))
        dxi(1,i) = 0.5 * vmax * (cos(min(alpha_ij)) + cos(max(alpha_ij))) ;
        dxi(2,i) = 0.5 * vmax * (sin(min(alpha_ij)) + sin(max(alpha_ij))) ;
    end
end