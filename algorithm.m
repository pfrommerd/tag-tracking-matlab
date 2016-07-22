function [  ] = algorithm(camParams, images)
    K = transpose(camParams.IntrinsicMatrix);
    
    if ~exist('images', 'var')
        images = VideoSource('video.mp4', camParams);
    end
    filter = Filter();
    
    tagSize = 0.1635;
    detector = TagDetector(tagSize, camParams);
    tracker = TagTracker();
    
    while images.hasImage()
        img = images.readImage();
        
        clf();

        imshow(img);
        hold on;
        
        tag = [];

        tracker_tags = tracker.process(img);
        if size(tracker_tags, 2) > 0
            tag = tracker_tags{1};
        end
        
        detector_tags = detector.process(img);
        if size(detector_tags, 2) > 0
            tag = detector_tags{1};
            tracker.track(K, tagSize, tag);
        end
        
        %tag
        
        if length(tag) > 0
            [x, P] = filter.step(tag, 1);

            % Draw tag
            drawTag(K, tagSize, x, 'r');
        end
        drawnow
    end
end

function drawTag(K, tagSize, tag, color)
    points = projectTag(K, tagSize, tag);
    
    x = points(1, :);
    y = points(2, :);
    
    plot([x x(1)], [y y(1)], '.-', 'Color', color);
end