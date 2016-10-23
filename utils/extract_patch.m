function p = extract_patch(K, patchSize, coords, img, tag)
    p = zeros(patchSize);
    
    % Create the homography
    R = quat_to_rotm(tag.state(4:7));
    T = tag.state(1:3);
    H = K * [R(:, 1:2) T];
    
    % Project the coords
    s = tag.size + tag.border;
    S = [s(1) 0 0; ...
         0 s(2) 0; ...
         0 0    1];
    % Modify H to scale by size first
    H = H * S;

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

