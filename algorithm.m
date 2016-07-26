function [ X, T, FT, NU  ] = algorithm(camParams, images)
    K = transpose(camParams.IntrinsicMatrix);
    tagSize = 0.1635;
    
    if ~exist('images', 'var')
        images = VideoSource('video.mp4', camParams);
    end
    
    A = @(x) timeUpdate(x, 1);
    H = @(x) measureTransform(K, tagSize, x);
    filter = Filter(A, H, [0; 0; 5; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0]);
    
    detector = TagDetector(tagSize, camParams);
    tracker = TagTracker();
    
    X = [];
    T = [];
    GT = [];
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
    
    
    time = tic;
    while images.hasImage() && time < time_limit
        %toc(time);
        %time = tic;

        img = images.readImage();
        
        set(0, 'CurrentFigure', fig1)
                
        tag = [];
        gt = [];
        filteredTag = [];

        %{
        tracker_tags = tracker.process(img);
        if size(tracker_tags, 2) > 0
            tag = tracker_tags{1};
        end
        
        detector_tags = detector.process(img);
        if size(detector_tags, 2) > 0
            tag = detector_tags{1};
            tracker.track(tag);
        end
        gt = Hinv(K, tagSize, tag);        
        %}
        
        % If we're debugging, feed in some generated data
        [tag, gt] = generateTag(K, tagSize, time);
        
        %toc(time)
        
        if length(tag) > 0
            t = Hinv(K, tagSize, tag);        

            time = time + 1;
            [x, P, nu] = filter.step(tag);
            
            % Convert the x to a tag
            filteredTag = measureTransform(K, tagSize, x);

            X = [X x];
            T = [T t];
            GT = [GT gt];
            NU = [NU nu];
        end
        
        %toc(time)

        clf(fig1);
        imshow(img);
        hold on;

        drawTag(tag, 'r');
        drawTag(filteredTag, 'blue');
        
        drawnow
        hold off;
        debug_plot(X, GT, T, NU, fig2, fig3, fig4, fig5, fig6);
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
    x(4:7, :) = [ones([1 size(x, 2)]); zeros([3 size(x, 2)])];
    z = projectTag(K, tagSize, x);
end

function x = Hinv(K, tagSize, z)
    tagSize = tagSize/2;
    pin = [-tagSize, -tagSize; ...
            tagSize, -tagSize; ...
            tagSize,  tagSize;...
           -tagSize,  tagSize];
         
    points = [ z([1, 3, 5, 7], :) z([2, 4, 6, 8], :) ];
    H = homography_solve(pin, points);

    [R, T] = homography_extract_pose(K, H);

    % Comute a quaternion from a rot mat
    rot = rotm_to_quat(R)';
    x = [ T; rot];
end

function [tag, x] = generateTag(K, tagSize, time)
    vel = 0.01;
    x = zeros(13, 1);
    
    % Set the position
    x(1:3) = [time * vel; 0; 5];
    
    % Rotate the tag around the y axis
    rVec = [0 time * 1 0];
    
    % Set the rotation
    x(4:7) = rotvec_to_quat(rVec);
    
    % Now project the points
    tag = measureTransform(K, tagSize, x);
    
    tag = tag + ( 0.01 * randn([8, 1]) );
end

function drawTag(tag, color)    
    x = tag([1, 3, 5, 7], :)';
    y = tag([2, 4, 6, 8], :)';
    
    plot(x(1), y(1), 'x', 'Color', color);
    plot(x(2), y(2), 'o', 'Color', color);
    plot(x(3), y(3), '*', 'Color', color);
    plot(x(4), y(4), 's', 'Color', color);
    %plot([x x(1)], [y y(1)], '.-', 'Color', color);
end