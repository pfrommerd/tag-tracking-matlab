function tags = projectTags(K, tagSize, x, id, color) % color is optional
    X = [[-tagSize/2; -tagSize/2] ...
         [ tagSize/2; -tagSize/2] ...
         [ tagSize/2;  tagSize/2] ...
         [-tagSize/2;  tagSize/2]];
     
    if ~exist('color', 'var')
        color = 'b';
    end
    
    tags = {};
    for i=1:size(x, 2)
        R = quat_to_rotm(x(4:7, i));
        T = x(1:3, i);
        H = K * [R(:, 1:2) T];

        corners = homography_project(H, X)';

        tags{i}.id = id;
        tags{i}.color = color;
        tags{i}.corners = corners;
    end
end