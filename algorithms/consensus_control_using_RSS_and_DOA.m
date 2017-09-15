function [dxi,stop_condition,energy] = consensus_control_using_RSS_and_DOA(L,xi)

global N desired_distance error_bearing error_distance;

dxi = zeros(2, N);
stop_condition = 1;
vmax = 2;
energy = 0;
C = -20; % Constant to represent the RSS at refenrece distance d0=1m (in dBm)
PLE = 3; % The Path loss exponent
shadowing_std = 4; % Standard deviation of the zero-mean Gaussian variable to represent the Shadow fading effects (in dBm)
distance_scale = 50; % Scale the robotarium boundaries to a higher value

for i = 1:N
    neighbors = topological_neighbors(L, i);
    xvel = zeros(1,length(neighbors));
    yvel = zeros(1,length(neighbors));
    %w_ij = zeros(1,length(neighbors));
    RSS_ij = zeros(1,length(neighbors));
    k=1;
    % Iterate through agent i's neighbors
    for j = neighbors
        alpha_ij = atan2(xi(2,j)-xi(2,i),xi(1,j)-xi(1,i)) + error_bearing*randn;
        % For each neighbor, calculate appropriate consensus term and
        %add it to the total velocity
        %dxi(:, i) = dxi(:, i) + (xi(:, j) - xi(:, i));
        p_ij = norm(xi(1:2, i) - xi(1:2, j)) ;
        RSS_ij(k) = C - 10*PLE*log10(max(distance_scale*p_ij + distance_scale*error_distance*randn,1)) + shadowing_std*randn; % simulating the RSS values of a higher-scale environment since robotarium boundaries are small (the RSS model used is a far-field model (>1m) and generalizes over a wide disntace range of any wireless device)
        %w_ij(k) = -RSS_ij; % weight values as pure RSS
        xvel(k) = cos(alpha_ij) ;
        yvel(k) = sin(alpha_ij) ;
        k = k+1;
        if (p_ij > desired_distance)
            stop_condition = 0;
        end
        energy = energy + p_ij;
    end
    %w_ij = 0.001* 10.^(-RSS_ij/10); % RSS measurement converted from dBm to mW (absolute power)
    w_ij = 0.001 *10.^(RSS_ij/10); % RSS measurement converted from dBm to mW (absolute power)
    w_ij = w_ij./sum(w_ij); % Normalize the weight vector for a given robot
  
    if(~isempty(neighbors))
        dxi(1,i) = vmax*sum(xvel./w_ij) / length(neighbors);
        dxi(2,i) = vmax*sum(yvel./w_ij) / length(neighbors);
    end
end