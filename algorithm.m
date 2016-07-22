function [ X, T, NU  ] = algorithm(camParams, images)
    K = transpose(camParams.IntrinsicMatrix);
    
    if ~exist('images', 'var')
        images = VideoSource('video.mp4', camParams);
    end
    filter = Filter();
    
    tagSize = 0.1635;
    detector = TagDetector(tagSize, camParams);
    tracker = TagTracker();
    
    X = [];
    T = [];
    NU = [];
    
    time = 0;
    time_limit = 200;
    
    lastTag = zeros(13, 1);
    lastTag(4:7) = [1 0 0 0];
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    fig5 = figure(5);
    fig6 = figure(6);
    figure(1);
    
    while images.hasImage() && time < time_limit
        img = images.readImage();
        
        set(0, 'CurrentFigure', fig1)
        
        clf(fig1);
        imshow(img);
        hold on;
        
        tag = [];

        tracker_tags = tracker.process(img, lastTag);
        if size(tracker_tags, 2) > 0
            tag = tracker_tags{1};
        end
        
        detector_tags = detector.process(img, lastTag);
        if size(detector_tags, 2) > 0
            tag = detector_tags{1};
            tracker.track(K, tagSize, tag);
        end
        
        % If we're debugging, feed in some generated data
        %tag = generateTag(time, lastTag);
        
        if length(tag) > 0
            lastTag = tag;

            time = time + 1;
            [x, P, nu] = filter.step(tag, 1);

            X = [X x];
            T = [T tag];
            NU = [NU nu];
            
            %x
            % Draw tag
            color = 'r';
            if size(detector_tags, 2) < 1
                color = 'y';
            end
            
            drawTag(K, tagSize, tag, color);
            drawTag(K, tagSize, x, 'blue');
        end

        drawnow
        hold off;
        debug_plot(X, T, NU, fig2, fig3, fig4, fig5, fig6);
        set(0, 'CurrentFigure', fig1)
    end
end

function tag = generateTag(time, lastTag)
    tag = zeros(13, 1);
    
    % Set the position
    tag(1:3) = [time * 0.01; 0; 5];
    %tag(1:3) = [0; 0; 5];
    
    % Rotate the tag around the y axis
    rVec = [0 time * 0.1 0];
    %rVec = [0 0 0];
    
    
    % Set the rotation
    tag(4:7) = rotvec_to_quat(rVec);

    % Apply noise
    tag = tag +( 0.000005 * randn([13, 1]) );

    % Set the velocity
    tag(8:10) = [0.01; 0; 0];
    %tag(8:10) = [0; 0; 0];

    % Set the rotational velocity
    tag(11:13) = [0; 0.1; 0];
    %tag(11:13) = [0; 0; 0];
end

function drawTag(K, tagSize, tag, color)
    points = projectTag(K, tagSize, tag);
    
    x = points(1, :);
    y = points(2, :);
    
    plot([x x(1)], [y y(1)], '.-', 'Color', color);
end