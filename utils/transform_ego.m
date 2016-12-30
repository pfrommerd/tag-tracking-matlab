function newPose = transform_ego(tagPose, x)
    % Local -> World transform
    WL = [[quat_to_rotm(tagPose(4:7)') [tagPose(1); tagPose(2); tagPose(3)]]; [0 0 0 1]];
    % World -> Camera transform
    CW = [[quat_to_rotm(x(4:7)') [x(1); x(2); x(3)]]; [0 0 0 1]];
    T = CW * WL;
    newPose = tagPose;
    newPose(1:3) = [T(1, 4) T(2, 4) T(3, 4)];
    newPose(4:7) = rotm_to_quat(T(1:3, 1:3));
end