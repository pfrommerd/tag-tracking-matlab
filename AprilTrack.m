classdef AprilTrack < TagSource
    %PARTICLEFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector
        process_noise
        
        particles
        weights
        refPatch
        
        K
        tagSize
        % The projection Coordinates
        projCoordinates
    end
    
    methods
        function obj = AprilTrack(K, tagSize, totalTagSize, patchSize)
            obj.detector = TagDetector(K, tagSize);
            obj.process_noise = [0.01 0.01 0.001 ...
                                 0.005 0.005 0.005 ...
                                 0.01 0.01 0.001 ...
                                 0.001 0.001 0.001];
            
            obj.K = K;
            obj.tagSize = tagSize;
            
            obj.particles = [];
            obj.weights = [];
            obj.refPatch = zeros(patchSize); % The last reference patch
            
            % Pre-calculate the projection coordinates
            coordinates = zeros([size(obj.refPatch) 2]);
            for i=1:size(obj.refPatch, 2)
                for j=1:size(obj.refPatch, 1)
                    idx = [i / size(obj.refPatch, 1) - 0.5; ...
                           j / size(obj.refPatch, 2) - 0.5];

                    coordinates(j, i, :) = [idx(1) * totalTagSize(1); ...
                                            idx(2) * totalTagSize(2)];
                end
            end
            
            obj.projCoordinates = coordinates;
        end
        
        function tags = process(this, img)
            detector_tags = this.detector.process(img);
                                            
            if length(detector_tags) > 0
                tag = detector_tags{1};
                x = calcState(this.K, this.tagSize, tag.corners);

                % Update the reference patch
                this.refPatch = ...
                    extractPatch(this.K, x, this.projCoordinates, img);

                %x(1) = x(1) + 0.1;
                if size(this.particles, 2) < 1
                    this.particles = repmat(x, 1, 500);

                    n = size(this.particles, 2);
                    this.weights =  1/n * ones([1 n]);
                end
            end
            [this.particles, this.weights] = ...
                resample_particles(size(this.particles, 2), ...
                                    this.particles, ...
                                    this.weights);
            this.particles = propagate_particles(this.particles, ...
                                                this.process_noise, 1);

            this.weights = calcWeights(this.K, this.projCoordinates, ...
                                    this.particles, img, this.refPatch);
            
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            x
                                
            tags = projectTags(this.K, this.tagSize, x, 0, 'r');
            %tags = projectTags(this.K, this.tagSize, this.particles, 0, 'r');
        end
        
        function debug_plot(this, fig1, fig2, fig3, img)
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            
            set(0, 'CurrentFigure', fig2);
            clf(fig2);
            imshow(this.refPatch);
            
            set(0, 'CurrentFigure', fig3);
            clf(fig3);
            p = extractPatch(this.K, x, this.projCoordinates, img);
            imshow(p);
            
            set(0, 'CurrentFigure', fig1);
            
            g_x = linspace(-0.5, 0.5, 50);
            g_y = linspace(-0.5, 0.5, 50);
            %g_x = linspace(0, 0.3, 30);
            %g_y = linspace(-0.1, 0.1, 30);
            [s_x, s_y] = meshgrid(g_x, g_y);
            s_x = s_x(:)';
            s_y = s_y(:)';
            one = ones(size(s_x));
            zero = zeros(size(s_x));

            gen_particles = [s_x; s_y; x(3) * one; one; zero; zero; zero; zero; ...
                                 zero; zero; zero; zero; zero];
            gen_weights = calcWeights(this.K, this.projCoordinates, ...
                                    gen_particles, img, this.refPatch);

            w  = reshape(gen_weights', [length(g_y) length(g_x)]);
            
            clf(fig1);
            surf(g_x, g_y, w);
            colormap('parula');

            % Plot the current particles on top
            hold on;
            
            plot3(this.particles(1, :), this.particles(2, :), this.weights, 'x', 'Color', 'g');
        end
    end
end

% Will calculate the weights for given particles X
function Z = calcWeights(K, projCoordinates, X, img, refPatch)
    Z = zeros([1 size(X, 2)]);
    for i=1:size(X, 2)
        p = extractPatch(K, X(:, i), projCoordinates, img);
        z = measureDiff(p, refPatch);
        Z(i) = z;
    end
end

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
    x = [ T; rot; 0; 0; 0; 0; 0; 0];
    %x = [ T; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];

end


% Utility functions for comparing patches

% Extracts a patch given an image
function p = extractPatch(K, x, coordinates, img)
    % Create the homography
    %x(4:7) = [1; 0; 0; 0];
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

    if length(find(x < 1)) > 0 || ...
       length(find(y < 1)) > 0 || ...
       length(find(x > size(img, 2))) > 0 || ...
       length(find(y > size(img, 1))) > 0
        
        p = [];
        return;
    end
    
    x(find(x < 1)) = 1;
    y(find(y < 1)) = 1;

    x(find(x > size(img, 2))) = size(img, 2);
    y(find(y > size(img, 1))) = size(img, 1);
    
    idx = sub2ind(size(img), y, x);
    v = img(idx);
    p = reshape(v, size(p))';
end


function z = measureDiff(patchA, patchB)
    if (size(patchA) ~= size(patchB))
        z = 0;
        return;
    end
        
    a = patchA(:);
    b = patchB(:);

    z = abs(corr2(a, b));
    if z > 1 || isnan(z) % Some crazy value, like +/-Inf
        z = 0;
    end
end


