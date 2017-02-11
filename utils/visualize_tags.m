function visualize_tags( tagsFile )
    tags = load_tag_config(tagsFile);
    
    % A matrix with each corner as a column vector
    %corners = [];
    clf
    hold on;
    
    %transformed_tags = zeros(length(tags), 10);
    for i=1:length(tags)
        tag = tags{i};
        tagSize = tag.size;
        
        R = quat_to_rotm(tag.state(4:7));
        T = tag.state(1:3);
        
        RT = [R T];

        points = [-tagSize(1)  tagSize(1) tagSize(1) -tagSize(1)  0;
                  -tagSize(2) -tagSize(2) tagSize(2)  tagSize(2)  0;
                  0            0          0           0           0;
                  1            1          1          1            1];
        transformed = RT * points;
        corners = [transformed(1:3, 1:4) transformed(1:3, 1)];
        % Matlab takes xyz, but we need to give it xzy so
        % that it plots in our CS
        plot3(corners(1, :), corners(3, :), corners(2, :), '-', 'Color', 'r')
        %plot3(corners(1, :), corners(2, :), corners(3, :), '-', 'Color', 'r')
        
        % Add floating text with the tag id in the center of the tag
        text(transformed(1, 5), transformed(3, 5), transformed(2, 5), num2str(tag.id));
    end
    
    axis([-8 8 -8 8 -8 8])
    set(gca,'zdir','reverse')
    xlabel('x');
    ylabel('z');
    zlabel('y');
end

