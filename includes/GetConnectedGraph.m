function [L,G] = GetConnectedGraph(xi,sensing_range)
%GetConnectedGraph computes the connected graph in terms of Graph object and its Laplacian given a set of node coordinates (xi - 2xN
%vector) and a sensing range for each node
%Author: Ramviyas Parasuraman, ramviyas@purdue.edu

N = length(xi); % Number of nodes
source_list = ones(1,N*N); % Variable to store source nodes of the edges - preallocated with max possible edges
destination_list = ones(1,N*N); % Variable to store destination nodes of the edges - preallocated with max possible edges
weights_list = ones(N*N);% Variable to store weights of the edges - preallocated with max possible edges
edge_count = 0; % Number of edges
for i=1:N
    neighbors_i = 0; % Number of neighbors for node i
    neighbors_index_i = i*ones(1,N); % Pre-allocating the neighbors of i
    neighbors_weight_i = i*ones(1,N); % Pre-allocating the neighbor weights of i

    for j=1:N
        dist_ij = norm(xi(:,i)-xi(:,j)) ;
        if(j~=i && dist_ij <= sensing_range)
            neighbors_i = neighbors_i + 1;
            neighbors_index_i(neighbors_i) = j;
            neighbors_weight_i(neighbors_i) = dist_ij;
        end
    end
    neighbors_index_i = neighbors_index_i(1:neighbors_i);
    neighbors_weight_i = neighbors_weight_i(1:neighbors_i);
    if(neighbors_i ~= 0) 
        source_list(edge_count+1:edge_count+neighbors_i) = i*ones(1,neighbors_i);
        destination_list(edge_count+1:edge_count+neighbors_i) = neighbors_index_i;
        weights_list(edge_count+1:edge_count+neighbors_i) = neighbors_weight_i;
    end
    edge_count = edge_count + neighbors_i;
end

source_list = source_list(1:edge_count);
destination_list = destination_list(1:edge_count);
weights_list = weights_list(1:edge_count);

EdgeList = [source_list(:), destination_list(:), weights_list(:)];
uniqueEdgeList = unique([sort(EdgeList(:,[1,2]), 2 ) EdgeList(:,3)], 'rows' );
G = graph(uniqueEdgeList(:,1),uniqueEdgeList(:,2),uniqueEdgeList(:,3),N);
L = full(laplacian(G));