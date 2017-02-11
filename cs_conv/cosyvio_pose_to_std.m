% Converts a cosyvio pose (where x = z_std, y = -x_std, z = -y_std) to a
% standard pose with the conversion
% A = [0 -1 0; 0 0 -1; 1 0 0];
% X_std_cam = A * X_cosyvio_cam
% X_std_world = B * X_cosyvio_world

% The cosyvio dataset uses the form
% X_c = R * X_w + T

% We use
% R * X_c + T = X_w

% it can be solved that therefore
% R_std = B * inv(R_cosyvio) * inv(A)
% and
% T_std = -B * inv(R_cosyvio) * T_cosyvio

function [ std ] = cosyvio_pose_to_std(cosvio)
    A = [1 0 0; 0 1 0; 0 0 1];
    B = [0 -1 0; 0 0 -1 ; 1 0 0];
    
    R_cos = [quat_to_rotm(cosvio(4:7))];
    T_cos = [cosvio(1); cosvio(2); cosvio(3)];
    
    R_std = B * inv(R_cos) * inv(A);
    T_std = - B * inv(R_cos) * T_cos;
    
    quat = rotm_to_quat(R_std);
    
    std = [T_std(1); T_std(2); T_std(3); quat'];
end