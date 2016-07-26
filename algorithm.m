%{
classdef Algorithm
    properties
    end
    methods
    end
end
%}

function [ M ] = algorithm(camParams, images)
    K = transpose(camParams.IntrinsicMatrix);
    tagSize = 0.1635;
    
    if ~exist('images', 'var')
        images = VideoSource('video.mp4', camParams);
    end
    
    A = @(x) timeUpdate(x, 1);
    H = @(x) measureTransform(K, tagSize, x);

    tagSource = FusedTagSource(tagSize, camParams, 4);
    filter = Filter(A, H, 4);
    init = false;

    %tagSource = DebugTagSource(H);
    
    X = [];
    T = [];
    NU = [];
    
    M = [];
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    figure(1);

    tic
    
    while images.hasImage()
        img = images.readImage();
        
        set(0, 'CurrentFigure', fig1)

        tag = [];
        filteredTag = [];

        tags = tagSource.process(img);
        if size(tags, 2) > 0
            tag = tags(:, 1);
        end
        
        % The ground truth, set this to zero for now
        z = [];
        %%{
        if length(tag) > 0
            t = Hinv(K, tagSize, tag);  

            x = [];
            if ~init     
                filter.setState(t);
                x = t;
                z = tag;
                nu = zeros([8, 1]);
                init = true;
            else
                [x, z, P, nu] = filter.step(tag);
            end
            
            % Convert the x to a tag
            filteredTag = measureTransform(K, tagSize, x);

            X = [X x];
            T = [T t];
            %NU = [NU nu];            
        end
        %}
        %toc(time)

        clf(fig1);
        imshow(img);
        hold on;

        drawTag(tag, 'r');
        drawTag(filteredTag, 'blue');

        drawTag(z, 'yellow');

        drawnow
        %M = [ M getframe ];
        
        hold off;
        debug_plot(X, T, NU, fig2, fig3, fig4);
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
    %x(4:7, :) = [ones([1 size(x, 2)]); zeros([3 size(x, 2)])];
    X = [[-tagSize/2; -tagSize/2; 0; 1] ...
         [ tagSize/2; -tagSize/2; 0; 1] ...
         [ tagSize/2;  tagSize/2; 0; 1] ...
         [-tagSize/2;  tagSize/2; 0; 1]];
     
    z = projectTag(K, tagSize, x, X);
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
    x = [ T; rot; 0; 0; 0; 0; 0; 0];
end

function drawTag(tag, color)    
    if max(size(tag)) < 1
        return
    end
    points = reshape(tag, [2, size(tag, 1) * size(tag, 2) / 2]);
    x = points(1, :);
    y = points(2, :);
    
    plot(x(1), y(1), 'x', 'Color', color);
    plot(x(2), y(2), 'o', 'Color', color);
    plot(x(3), y(3), '*', 'Color', color);
    plot(x(4:end), y(4:end), 's', 'Color', color);
end