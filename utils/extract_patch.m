function p = extract_patch(K, patchSize, coords, img, tag, transState)
    p = zeros(patchSize);
    
    % Create the homography
    R = quat_to_rotm(transState(4:7));
    T = transState(1:3);
    H = K * [R(:, 1:2) T];
    
    % Project the coords
    s = tag.size + tag.border;
    
    % Modify H to scale by size first
    H = H * [s(1) 0 0; ...
             0 s(2) 0;
             0 0 1];
        

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

