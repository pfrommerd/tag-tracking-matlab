function [ X, T, FT, NU  ] = algorithm(camParams, images)
    K = transpose(camParams.IntrinsicMatrix);
    tagSize = 0.1635;
    
    if ~exist('images', 'var')
        images = VideoSource('video.mp4', camParams);
    end
    
    A = @(x) timeUpdate(x, 1);
    H = @(x) measureTransform(K, tagSize, x);
    filter = Filter(A, H, [0; 0; 10; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
    filter.setPos([0; 0; 5; 1; 0; 0; 0]);
    
    detector = TagDetector(tagSize, camParams);
    tracker = TagTracker();
    
    X = [];
    T = [];
    FT = [];
    NU = [];
    
    time = 0;
    time_limit = 1000;
    
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

        tracker_tags = tracker.process(img);
        if size(tracker_tags, 2) > 0
            tag = tracker_tags{1};
        end
        
        detector_tags = detector.process(img);
        if size(detector_tags, 2) > 0
            tag = detector_tags{1};
            %corr_x = Hinv(K, tagSize, tag);
            %corr_x
            %filter.setPos(corr_x);
            tracker.track(tag);
        end
        
        % If we're debugging, feed in some generated data
        tag = generateTag(K, tagSize, time);
        
        if length(tag) > 0
            time = time + 1;
            [x, P, nu] = filter.step(tag);
            % Convert the x to a tag
            filteredTag = measureTransform(K, tagSize, x);

            X = [X x];
            T = [T tag];
            FT = [FT filteredTag];
            NU = [NU nu];
            
            %x
            % Draw tag
            color = 'r';
            if size(detector_tags, 2) < 1
                color = 'y';
            end
            
            drawTag(tag, color);
            drawTag(filteredTag, 'blue');
        end

        drawnow
        hold off;
        debug_plot(X, T, FT, NU, fig2, fig3, fig4, fig5, fig6);
        set(0, 'CurrentFigure', fig1)
    end
end

% Time update function
function xp = timeUpdate(x, deltaT)
    xp = x;
    for i=1:size(x, 2)
        % Update the position and the orientation
        xp(1:3, i) = x(1:3, i) + deltaT * x(8:10, i); % velocity
        xp(4:7, i) = qmult(x(4:7, i)', rotvec_to_quat((x(11:13, i) * deltaT)')); % angular velocity
    end
end

function z = measureTransform(K, tagSize, x)
    x(4:7) = [1; 0; 0; 0];
    z = projectTag(K, tagSize, x);
end

function x = Hinv(K, tagSize, z)
    tagSize = tagSize/2;
    pin = [-tagSize, tagSize; ...
             tagSize tagSize; ...
             tagSize -tagSize;...
             -tagSize -tagSize];
         
    points = [ z([1, 3, 5, 7], :) z([2, 4, 6, 8], :) ];
    H = homography_solve(pin, points);

    [R, T] = homography_extract_pose(K, H);

    % Comute a quaternion from a rot mat
    rot = rotm_to_quat(R)';
    x = [ T; rot];
end

function tag = generateTag(K, tagSize, time)
    vel = 0.01;
    x = zeros(13, 1);
    
    % Set the position
    x(1:3) = [time * vel; 0; 5];
    
    % Rotate the tag around the y axis
    rVec = [0 time * 0.1 0];
    
    % Set the rotation
    x(4:7) = rotvec_to_quat(rVec);
    
    % Now project the points
    tag = measureTransform(K, tagSize, x);
    
    %tag = tag + ( 0.1 * randn([8, 1]) );
end

function drawTag(tag, color)    
    x = tag([1, 3, 5, 7], :)';
    y = tag([2, 4, 6, 8], :)';
    
    plot([x x(1)], [y y(1)], '.-', 'Color', color);
end