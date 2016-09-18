function tags = project_tags(K, tags)
    for i=1:length(tags)
        x = tags{i}.state;
        
        tagSize = tags{i}.size;
        X = [[-tagSize(1)/2; -tagSize(2)/2; 1] ...
             [ tagSize(1)/2; -tagSize(2)/2; 1] ...
             [ tagSize(1)/2;  tagSize(2)/2; 1] ...
             [-tagSize(1)/2;  tagSize(2)/2; 1]];

        R = quat_to_rotm(x(4:7));
        T = x(1:3);
        H = K * [R(:, 1:2) T];

        corners = homography_project(H, X)';

        tags{i}.corners = corners;
    end
end