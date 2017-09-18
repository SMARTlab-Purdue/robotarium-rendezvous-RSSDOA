function [dxi,stop_condition,energy] = bearing_only_rendezvous_using_all_bearings(L,xi)
% This function implements a linear Bearings-only controller using all bearing
% measurements as the control inputs based on the following reference:
% Zheng, R., & Sun, D. (2013). Rendezvous of unicycles: A bearings-only and perimeter shortening approach. Systems & Control Letters, 62(5), 401-407.
% autonomous agents using nearest neighbor rules,” IEEE Trans. Autom. Control, vol. 48, no. 6, pp. 988–1001, Jun. 2003.
% Author of this code: Ramviyas Parasuraman. ramviyas@purdue.edu

global N desired_distance error_bearing;

dxi = zeros(2, N);
stop_condition = 1;
vmax = 2;
energy = 0; % Lyapunov candidate function 

for i = 1:N
    neighbors = topological_neighbors(L, i);
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn; % adding noise to the bearing measurements
        % For each neighbor, calculate velocities term based on bearings
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