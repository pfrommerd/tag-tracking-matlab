function p = extract_patch(K, patchSize, coords, img, x)
    p = zeros(patchSize);
    
    % Create the homography
    R = quat_to_rotm(x(4:7));
    T = x(1:3);
    H = K * [R(:, 1:2) T];
    
    % Project the coords
    c = homography_project(H, coords);

    x = round(c(1, :));
    y = round(c(2, :));

    if length(find(x < 1)) > 0 || ...
       length(find(y < 1)) > 0 || ...
       length(find(x > size(img, 2))) > 0 || ...
       length(find(y > size(img, 1))) > 0
        
        p = [];
        return;
    end
    
    idx = sub2ind(size(img), y, x);
    v = img(idx);
    p = reshape(v, size(p))';
end

