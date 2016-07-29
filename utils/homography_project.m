
function [ x ] = homography_project(H, X)
    % X is 2xN, so add an extra row of ones
    X = [X; ones([1 size(X, 2)])];

    t = H * X;
    % Divide by the last row
    t = bsxfun(@rdivide, t, t(3,:));
    
    % Extract the first two rows
    x = t(1:2, :);
end

