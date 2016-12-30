function [ H ] = homography_from_state(K, tag)

    R = quat_to_rotm(tag.state(4:7));
    T = tag.state(1:3);
    H = K * [R(:, 1:2) T];
    % Create the homography
    
    % Project the coords
    s = tag.size + tag.border;
    S = [s(1) 0 0; ...
         0 s(2) 0; ...
         0 0    1];
    % Modify H to scale by size first
    H = H * S;
end