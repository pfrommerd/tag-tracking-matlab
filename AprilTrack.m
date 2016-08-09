classdef AprilTrack < TagSource
    %PARTICLEFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector
        
        particles
        weights
        measurements
        
        refPatch
                
        num_particles
        lambda
        process_noise
        K
        tagSize
        
        % The projection coordinates cached
        projCoordinates
    end
    
    methods
        function obj = AprilTrack(params)
            obj.detector = TagDetector(params.K, params.tagSize);

            obj.num_particles = params.num_particles;
            obj.lambda = params.lambda;
            obj.process_noise = params.process_noise;
            obj.K = params.K;
            obj.tagSize = params.tagSize;
            
            obj.particles = [];
            obj.weights = [];
            obj.refPatch = zeros(params.patchSize); % The last reference patch
            
            % Pre-calculate the projection coordinates            
            totalTagSize = params.patchTagSize;
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
        
        function tags = process(this, img, det_img)
            detector_tags = this.detector.process(det_img);
                                
            if length(detector_tags) > 0
                tag = detector_tags{1};
                x = calcState(this.K, this.tagSize, tag.corners);
                % Update the reference patch
                this.refPatch = ...
                    extractPatch(this.K, x, this.projCoordinates, img);

                %x(1) = x(1) + 0.1;
                %x(4:7) = [1 0 0 0];
                if size(this.particles, 2) < 1
                    this.particles = repmat(x, 1, this.num_particles);

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

            this.measurements = calcMeasurements(this.K, this.projCoordinates, ...
                                            this.particles, img, this.refPatch, this.lambda);
            
            m = normalize(this.measurements);
            this.weights = this.weights .* m;
            this.weights = normalize(this.weights);
            
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            x
            
            tags = projectTags(this.K, this.tagSize, x, 0, 'r');
        end
        
        function debug_plot(this, fig1, fig2, fig3, fig4, img)
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            
            set(0, 'CurrentFigure', fig3);
            clf(fig3);
            colormap(fig3, gray(256));
            image(this.refPatch);
            
            set(0, 'CurrentFigure', fig4);
            clf(fig4);
            colormap(fig4, gray(256));
            p = extractPatch(this.K, x, this.projCoordinates, img);
            image(p);
           
            
            this.debug_plot_pos(fig1, x, img);
            this.debug_plot_rot(fig2, x, img);
        end
        function debug_plot_pos(this, fig, x, img)
            set(0, 'CurrentFigure', fig);
            clf(fig);
            
            g_x = linspace(-0.5, 0.5, 3);
            g_y = linspace(-0.5, 0.5, 3);
            g_z = linspace(0.2, 0.7, 3);

            [s_x, s_y, s_z] = meshgrid(g_x, g_y, g_z);
            s_x = s_x(:);
            s_y = s_y(:);
            s_z = s_z(:);

            gen_particles = repmat(x, 1, size(s_z, 1));
            gen_particles(1:3, :) = [s_x, s_y, s_z]';
            
            
            gen_weights = calcMeasurements(this.K, this.projCoordinates, ...
                                    gen_particles, img, this.refPatch, this.lambda);
            
            gen_particles = [ gen_particles this.particles ];
            gen_weights = [ gen_weights this.measurements ];
            
            colormap('parula');
            caxis([0 1])
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_particles(3, :), 5, gen_weights);
        end
        
        function debug_plot_rot(this, fig, x, img)
            set(0, 'CurrentFigure', fig);
            clf(fig);
            
            % Plots angular measurements
            g_x = linspace(-10, 10, 5);
            g_y = linspace(-10, 10, 5);
            g_z = linspace(-10, 10, 5);
            [s_x, s_y, s_z] = meshgrid(g_x, g_y, g_z);
            s_x = s_x(:);
            s_y = s_y(:);
            s_z = s_z(:);

            g_rotv = [s_x s_y s_z];
            g_quat = rotvec_to_quat(g_rotv);
            
            % Augment with the particle set
            gp_quat = [g_quat; this.particles(4:7, :)'];
            
            fwd = repmat([0 0 -1 0], size(gp_quat, 1), 1);
            g_norm = qmult(qmult(gp_quat, fwd), qinv(gp_quat));
            
            gen_particles = repmat(x, 1, size(g_quat, 1));
            gen_particles(4:7, :) = g_quat';
            gen_z = calcMeasurements(this.K, this.projCoordinates, ...
                        gen_particles, img, this.refPatch, this.lambda);
            gen_z = [ gen_z this.measurements ];
                    
            g_norm = bsxfun(@times, g_norm, (gen_z' + 1));

            colormap('parula');
            caxis([0 1])
            scatter3(g_norm(:, 2)', g_norm(:, 3)', g_norm(:, 4)', 5, gen_z);
        end
    end
end

% Will calculate the weights for given particles X
function Z = calcMeasurements(K, projCoordinates, X, img, refPatch, lambda)
    Z = zeros([1 size(X, 2)]);
    for i=1:size(X, 2)
        p = extractPatch(K, X(:, i), projCoordinates, img);
        z = measureDiff(p, refPatch, lambda);
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

function z = measureDiff(patchA, patchB, lambda)
    if (size(patchA) ~= size(patchB))
        z = 0;
        return;
    else
        a = int16(patchA);
        b = int16(patchB);

        z = 1 - abs(corr2(a, b));
        if z > 1 || isnan(z) % Some crazy value, like -Inf
            z = 1;
        end
    end
    z = exp(-lambda * z);
end

% This version used SAD
%{
function z = measureDiff(patchA, patchB)
    if (size(patchA) ~= size(patchB))
        z = 0;
        return;
    else
        a = int16(patchA);
        b = int16(patchB);

        %z = 1 - abs(corr2(a, b));
        d = (a-b).^2;
        z = sum(d(:));
        if z < 0 || isnan(z) % Some crazy value, like -Inf
            z = 0;
        end
    end
    z = exp(-0.000001 * z);
end
%}

function wn = normalize(w)
    wn = w ./ sum(w);
end
