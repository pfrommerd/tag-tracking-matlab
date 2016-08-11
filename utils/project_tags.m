function tags = project_tags(K, tagSize, tags) % color is optional
    X = [[-tagSize(1)/2; -tagSize(2)/2; 1] ...
         [ tagSize(1)/2; -tagSize(2)/2; 1] ...
         [ tagSize(1)/2;  tagSize(2)/2; 1] ...
         [-tagSize(1)/2;  tagSize(2)/2; 1]];

    for i=1:length(tags)
        x = tags{i}.state;
        
        R = quat_to_rotm(x(4:7, i));
        T = x(1:3, i);
        H = K * [R(:, 1:2) T];

        corners = homography_project(H, X)';

        tags{i}.corners = corners;
    end
end