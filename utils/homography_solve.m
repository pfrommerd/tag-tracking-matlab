function [ H ] = homography_solve(in_pts, out_pts)
    % est_homography estimates the homography to transform each of the
    % in_pts to out_pts
    % Inputs:
    %     in_pts: a 4x2 matrix of corner points in the video
    %     out_pts: a 4x2 matrix of logo points that correspond to video_pts
    % Outputs:
    %     H: a 3x3 homography matrix such that outpts ~ H*video_pts

    A = [];

    for p=1:size(in_pts, 1)
        i = in_pts(p,:);
        o = out_pts(p,:);
        a_x = [ -i(1) -i(2) -1 0 0 0 i(1) * o(1) i(2) * o(1) o(1) ];
        a_y = [ 0 0 0 -i(1) -i(2) -1 i(1) * o(2) i(2) * o(2) o(2) ];
        A = [A; a_x; a_y];
    end

    [U, S, V] = svd(A);

    H = V(:, end);
    

    H = transpose(reshape(H, 3, 3));
    
    % H33 (Tz) must be positive
    % if it is negative, take the negative of the matrix
    % as H is only known up to a scale
    
    if H(3, 3) < 0
        H  = -1 * H;
    end
end