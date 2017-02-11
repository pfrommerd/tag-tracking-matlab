function visualize_poses(poses)
    clf;
    hold on;

    set(gca,'zdir','reverse')
    
    locations = [];
    
    for i=1:size(poses,2)
        pose = poses(:, i);
        
        loc = [pose(1) pose(2) pose(3)];
        rot = quat_to_rotm(pose(4:7)');
        
        % Do the matlab CS transform doohicky
        loc_matlab = [loc(1) loc(3) loc(2)];
        locations = [locations loc_matlab'];
        
        plotCamera('Location', loc_matlab, 'Orientation', rot, 'Size', 0.005, 'Opacity', 0);
    end
    plot3(locations(1, :), locations(2, :), locations(3, :), 'x-');

    set(gca,'zdir','reverse')
    xlabel('x');
    ylabel('z');
    zlabel('y');
    view(3);
end

