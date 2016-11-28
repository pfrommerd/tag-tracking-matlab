function [ R, T ] = homography_extract_pose(K, H_l)
    H_l = inv(K) * H_l;
    
    [U, D, V] = svd(H_l);
    
    H = H_l / D(2, 2); % Normalize
    
    [U, D, V] = svd(H' * H);
    
    u_1 = (sqrt(1 - D(3,3)^2) * V(:, 1) + sqrt(D(1, 1)^2 -1) * V(:, 3))/sqrt(D(1, 1)^2 - D(3, 3)^2);
    u_2 = (sqrt(1 - D(3,3)^2) * V(:, 1) - sqrt(D(1, 1)^2 -1) * V(:, 3))/sqrt(D(1, 1)^2 - D(3, 3)^2);

    U_1 = [V(:, 2) u_1 cross(V(:, 2), u_1)];
    U_2 = [V(:, 2) u_2 cross(V(:, 2), u_2)];
    
    W_1 = [H * V(:, 2) H * u_1 cross(H * V(:, 2), H * u_1)];
    W_2 = [H * V(:, 2) H * u_2 cross(H * V(:, 2), H * u_2)];
    
    N_1 = cross(V(:, 2), u_1);
    N_2 = cross(V(:, 2), u_2);
    
    R_1 = W_1 * U_1';
    R_2 = W_2 * U_2';
    
    T_1 = (H - R_1) * N_1;
    T_2 = (H - R_2) * N_2;
    
    % Enforce in front of camera to get rid of two solutions
    if T_1(3) < 0
        T_1 = -T_1;
        N_1 = -N_1;
    end
    
    % Enforce in front of camera to get rid of two solutions
    if T_2(3) < 0
        T_2 = -T_2;
        N_2 = -N_2;
    end
    
    S_1 = [R_1 T_1];
    S_2 = [R_2 T_2];
    
    R = R_2;
    T = T_2;
end