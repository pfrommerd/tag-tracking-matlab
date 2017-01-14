
function [ x ] = homography_project(H, X)
    t = H * X;
    
    if any(t(3,:) <= 0)
        x = ones([2 size(t,2)]) * -1;
    end
    
    % Divide by the last row
    x_x = t(1, :) ./ t(3, :);
    x_y = t(2, :) ./ t(3, :);

    x = [x_x; x_y];
end

