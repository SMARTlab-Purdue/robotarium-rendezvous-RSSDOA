function r = initialize_robot_positions(r,initial_positions)
%%Initialize the robots to a fixed position

global si_pos_controller si_to_uni_dyn N;
for iter=1:1000
    x = r.get_poses();
    %xi = uni_to_si_states(x);
    dxi = zeros(2, N);
    for i=1:N
        dxi(:, i) =  si_pos_controller(x(1:2, i), initial_positions(:,i));
    end
    dxu = si_to_uni_dyn(dxi, x);
    r.set_velocities(1:N, dxu);    
    r.step();
end
%r.get_poses();
%r.step();
%%