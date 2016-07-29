function [ M ] = corner_tracking(K, images)
    tagSize = 0.1635;
    % totalTagSize includes the white border
    %totalTagSize = [0.1935 0.1935];
    totalTagSize = [0.2 0.2];
    
    X = [];
    T = [];
    NU = [];
    
    M = [];
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    figure(1);
    
    % The last known good patch
    refPatch = zeros(32, 32);
    
    % Generate coordinates for sampling into patches
    coordinates = zeros([size(refPatch, 1) size(refPatch, 2) 2]);
    for i=1:size(refPatch, 2)
        for j=1:size(refPatch, 1)
            idx = [i / size(refPatch, 1) - 0.5; ...
                   j / size(refPatch, 2) - 0.5];
               
            coordinates(j, i, :) = [idx(1) * totalTagSize(1); ...
                                    idx(2) * totalTagSize(2)];
        end
    end
    
    % The last image
    img = [];

    tagSource = TagDetector(tagSize, K, 4);
    filter = Filter();
    init = false;
    
    while images.hasImage()
        img = images.readImage();
        
        set(0, 'CurrentFigure', fig1)

        tags = tagSource.process(img);
        if length(tags) > 0
            tag = tags{1};
            tagState = calcState(K, tagSize, tag.corners);
            if ~init
                tagState(1) = tagState(1) + 0.05;
                filter.setState(tagState);
                init = true;
            end
            refPatch = extractPatch(K, tagState, coordinates, img);
        end

        % Approximate the measurement (truePatch) to the referencePatch

        A = @(x) timeUpdate(x, 1);
        H = @(x) measureTransform(K, x, coordinates, img, refPatch);

        
        z = 1;% measureDiff(truePatch, refPatch);
        [x, y_out, z_out, z_minus, P, nu] = filter.step(z, A, H);
        
        t = projectTags(K, tagSize, x, 0, 'b');
        y_t = projectTags(K, tagSize, y_out, 0, [0 0 0]);
        % Label the y's with the correct color
        for i=1:length(y_t)
            z = z_out(i);
            y_t{i}.color = [0 z 0];
        end
        
        clf(fig1);
        image(img);
        hold on;

        drawTags(tags);
        drawTags(y_t);
        drawTags(t);
        drawnow;

        set(0, 'CurrentFigure', fig2)
        % Draw the last patch
        clf(fig2);
        image(refPatch);
        drawnow;
        % Generate sigmas that cover the whole distribution
        %%{
        g_x = linspace(0, -0.2, 30);
        g_y = linspace(0, -0.2, 30);
        [s_x, s_y] = meshgrid(g_x, g_y);
        s_x = s_x(:)';
        s_y = s_y(:)';
        one = ones(size(s_x));
        zero = zeros(size(s_x));
                
        gen_sigmas = [s_x; s_y; x(3) * one; one; zero; zero; zero; zero; ...
                             zero; zero; zero; zero; zero];
        gen_z = H(gen_sigmas);
        
        %gen_sigmas = [gen_sigmas y_out];
        %gen_z = [gen_z z_out];
        
        plot_sigmas(gen_sigmas, gen_z, g_x, g_y, fig3);
        hold on;
        plot3(x(1), x(2), z_minus, 'x', 'Color', 'r');
        plot3(y_out(1, :), y_out(2, :), z_out, 'o', 'Color', 'g');
        %}
        
        %debug_plot(X, T, NU, fig2, fig3, fig4);
        set(0, 'CurrentFigure', fig1)
    end
end

% Backs the state out of corners using a homography
function x = calcState(K, tagSize, corners)
    tagSize = tagSize/2;
    pin = [-tagSize, -tagSize; ...
            tagSize, -tagSize; ...
            tagSize,  tagSize;...
           -tagSize,  tagSize];
         
    H = homography_solve(pin, corners);
    
    % Extract the rotation and translation
    [R, T] = homography_extract_pose(K, H);

    % Comute a quaternion from a rot mat
    rot = rotm_to_quat(R)';
    % Assume zero velocity/rot velocity
    % as we've detected the tag
    % so there probably isn't much movement
    %x = [ T; rot; 0; 0; 0; 0; 0; 0];
    x = [ T; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
end

% Extracts a patch given an image
function p = extractPatch(K, x, coordinates, img)
    % Create the homography
    x(4:7) = [1; 0; 0; 0];
    R = quat_to_rotm(x(4:7));
    T = x(1:3);
    H = K * [R(:, 1:2) T];

    p = zeros([size(coordinates,1) size(coordinates, 2)]);
    X = reshape(permute(coordinates, [3, 2, 1]), ...
                2, size(p, 1) * size(p, 2));
    c = homography_project(H, X);
    c = round(c);
    
    x = c(1, :);
    y = c(2, :);

    x(find(x < 1)) = 1;
    y(find(y < 1)) = 1;

    x(find(x > size(img, 2))) = size(img, 2);
    y(find(y > size(img, 1))) = size(img, 1);
    
    idx = sub2ind(size(img), y, x);
    v = img(idx);
    p = reshape(v, size(p))';
end

% Time update function (A)
function xp = timeUpdate(x, deltaT)
    xp = x;
    for i=1:size(x, 2)
        % Update the position and the orientation
        % velocity
        xp(1:3, i) = x(1:3, i) + deltaT * x(8:10, i); 
        % angular velocity
        xp(4:7, i) = qmult(x(4:7, i)', rotvec_to_quat((x(11:13, i) * deltaT)')); 
        
        % Constrain the position so that we don't move through the camera
        % and end up with a false minimum at z = -infinity or anything
        % funky like that
        xp(3, i) = max(xp(3, i), 0);
    end
    xp = x;
end

% H function
% will take the cross correlation between the patch
% represented by x in the current image and the reference patch
function Z = measureTransform(K, X, coordinates, img, refPatch)
    Z = zeros([1 size(X, 2)]);
    for i=1:size(X, 2)
        p = extractPatch(K, X(:, i), coordinates, img);
        z = measureDiff(p, refPatch);
        Z(i) = z;
    end
end

function z = measureDiff(patchA, patchB)
    a = patchA(:);
    b = patchB(:);

    z = abs(corr2(a, b));
    if z > 1 || isnan(z) % Some crazy value, like +/-Inf
        z = 0;
    end
end

function drawTags(tags)
    for i=1:length(tags)
        drawTag(tags{i});
    end
end

% Draws a tag (the corner points, in column vector form)
function drawTag(tag)
    if max(size(tag)) < 1
        return
    end
    color = tag.color;
    points = tag.corners';
    x = points(1, :);
    y = points(2, :);
    
    plot([x x(1)], [y y(1)], '.-', 'Color', color);
    
    %{
    plot(x(1), y(1), 'x', 'Color', color);
    plot(x(2), y(2), 'o', 'Color', color);
    plot(x(3), y(3), '*', 'Color', color);
    plot(x(4:end), y(4:end), 's', 'Color', color);
    %}
end