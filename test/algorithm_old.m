function [  ] = algorithm_old( frames, num_frames, camParams )
    intrinsics = camParams.IntrinsicMatrix;
    params = [intrinsics(1, 1) intrinsics(2, 2) intrinsics(1, 3) intrinsics(2, 3)];
    
    pointTracker = vision.PointTracker;
    
    initialized = false;
    
    for i = 1:num_frames
        frame = frames{i};

        trackedPoints = [];
        if initialized
            trackedPoints = step(pointTracker, frame);
        end
        

        % Each tag is 0.1635m in one dimension
        results = find_apriltags(frame, 0.1635 , params);
        resultsSize = size(results);
        
        imshow(frame);
        hold on;

        if resultsSize(2) > 0
            % Process the tags and reset the tracker
            for i = 1:resultsSize(2)
                tag = results(1, i);

                x = tag.corners(1, :);
                y = tag.corners(2, :);
                points = transpose([x;y]);

                if ~initialized
                    initialize(pointTracker, points, frame);
                    initialized = true;
                else
                    % Otherwise reset the points
                    setPoints(pointTracker, points);
                end
                
                plot([x, x(1)], [y y(1)], '.-', 'Color', 'r')
            end
        else
            % Show the tracker points
            if initialized
                x = transpose(trackedPoints(:, 1));
                y = transpose(trackedPoints(:, 2));
                plot([x, x(1)], [y y(1)], '.-', 'Color', 'y')                
            end
        end
        drawnow
    end
end

