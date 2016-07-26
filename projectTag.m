function x = projectTag(K, tagSize, tags, X) % X is optional
    if ~exist('X', 'var')
        X = [[-tagSize/2; -tagSize/2; 0; 1] ...
             [ tagSize/2; -tagSize/2; 0; 1] ...
             [ tagSize/2;  tagSize/2; 0; 1] ...
             [-tagSize/2;  tagSize/2; 0; 1]];
    end
    
    x = zeros(size(X, 2) * 2, size(tags,2));
          
    for i=1:size(tags, 2)
        trans = tags(1:3, i);
        rot = tags(4:7, i);

        RT = [ quat_to_rotm(rot) trans];
        % Transformed X
        t = K * RT * X;
        t = bsxfun(@rdivide, t, t(3,:));
        
        % Stack the points into a single vector
        x(:, i) = reshape(t(1:2, :), [size(X, 2) * 2, 1]);
    end
end