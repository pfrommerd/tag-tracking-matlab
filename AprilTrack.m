classdef AprilTrack < TagSource
    %PARTICLEFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector
        
        x
        
        particles
        weights
        measurements
        
        refPatch
                
        num_particles
        lambda
        k
        alpha
        process_noise
        
        K
        tagSize
        patchSize
        totalTagSize

        
        % The projection coordinates cached
        projCoordinates
    end
    
    methods
        function obj = AprilTrack(params)
            obj.detector = TagDetector(params.K, params.tagSize);

            obj.x = zeros([13 1]);
            
            obj.num_particles = params.num_particles;
            obj.lambda = params.lambda;
            obj.k = params.k;
            obj.alpha = params.alpha;
            obj.process_noise = params.process_noise;
            obj.K = params.K;
            obj.tagSize = params.tagSize;
            obj.totalTagSize = params.patchTagSize;
            obj.patchSize = params.patchSize;
            
            obj.particles = [];
            obj.weights = [];
            obj.refPatch = zeros(params.patchSize); % The last reference patch
            
            % Pre-calculate the projection coordinates                        
            totalTagSize = obj.totalTagSize;
            coordinates = zeros([size(obj.refPatch) 2]);
            for i=1:size(obj.refPatch, 2)
                for j=1:size(obj.refPatch, 1)
                    idx = [i / size(obj.refPatch, 1) - 0.5; ...
                           j / size(obj.refPatch, 2) - 0.5];

                    coordinates(j, i, :) = [idx(1) * totalTagSize(1); ...
                                            idx(2) * totalTagSize(2)];
                end
            end
            obj.projCoordinates = reshape(permute(coordinates, [3, 2, 1]), ...
                                    2, size(obj.refPatch, 1) * size(obj.refPatch, 2));
            obj.projCoordinates = [obj.projCoordinates; ones([1 size(obj.projCoordinates, 2)])];
        end
        
        function tags = process(this, img, det_img)
            detector_tags = this.detector.process(det_img);
                                
            if length(detector_tags) > 0
                tag = detector_tags{1};
                if size(this.particles, 2) < 1
                    x = calcState(this.K, this.tagSize, tag.corners);
                    %x(4:7) = [1 0 0 0];
                    % Update the reference patch
                    this.refPatch = ...
                        extractPatch(this.K, this.projCoordinates, this.patchSize, x, img);

                    %x(1) = x(1) + 0.1;
                    %if size(this.particles, 2) < 1
                    this.particles = repmat(x, 1, this.num_particles);
                    
                    n = size(this.particles, 2);
                    this.weights =  1/n * ones([1 n]);
               end
            end

            [this.particles, this.weights] = ...
                resample_particles(size(this.particles, 2), ...
                                    this.particles, ...
                                    this.weights);

            % Propagate through q
            this.particles = q_smpl(this.particles, this.weights, 1, ...
                                    this.process_noise, this.k, this.alpha);
            % Measure the y's
            this.measurements = calcMeasurements(this.K, this.projCoordinates, ...
                                            this.particles, img, this.refPatch);
            % Update the weights
            m = normalize(convMeasurement(this.measurements, this.lambda));

            % q(x) = 1/(k + alpha*w_prev) * p(x)
            % so
            % p(x)/q(x) = alpha*w_prev + k
            this.weights = this.weights .* m .* (this.alpha * this.weights + this.k);
            this.weights = normalize(this.weights);
            
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            %x
            d = x - this.x;
            x(8:10)
            d(1:3)
            verr = x(8:10) - d(1:3)
            this.x = x;
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
            p = extractPatch(this.K, this.projCoordinates, this.patchSize, x, img);
            image(p);
           
            
            this.debug_plot_pos(fig1, x, img);
            this.debug_plot_rot(fig2, x, img);
        end
        function debug_plot_pos(this, fig, x, img)
            set(0, 'CurrentFigure', fig);
            clf(fig);
  
            g_x = linspace(-1, 1, 3);
            g_y = linspace(-1, 1, 3);
            g_z = linspace(0, 5, 3);
            [s_x, s_y, s_z] = meshgrid(g_x, g_y, g_z);
            s_x = s_x(:);
            s_y = s_y(:);
            s_z = s_z(:);
            
            gen_particles = [ this.particles(1:3, :) [s_x s_y s_z]' ];
            
            gen_w = [ this.weights ];
            gen_m = [ convMeasurement(this.measurements, this.lambda) zeros([1, size(s_x, 1)])];
            
            colormap('parula');
            caxis([0 1])
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_particles(3, :), 5, gen_m);
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
            gen_z = zeros([1, size(gen_particles, 2)]);
            
            gen_w = [ gen_z this.weights ];
            gen_m = [ gen_z convMeasurement(this.measurements, this.lambda) ];

            g_norm = bsxfun(@times, g_norm, (gen_m' + 1));

            colormap('parula');
            caxis([0 1])
            scatter3(g_norm(:, 2)', g_norm(:, 3)', g_norm(:, 4)', 5, gen_m);
        end
    end
end

% Will calculate the weights for given particles X
function Z = calcMeasurements(K, projCoordinates, X, img, refPatch)
    Z = zeros([1 size(X, 2)]);
    for i=1:size(X, 2)
        p = extractPatch(K, projCoordinates, size(refPatch), X(:, i),  img);
        z = measureDiff(p, refPatch);
        Z(i) = z;
    end
end

function w = convMeasurement(m, lambda)
    w = exp(-lambda * (1 - m));
    %w = m;
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
function p = extractPatch(K, X, patchSize, x, img)
    % The patch
    p = zeros(patchSize);

    % Create the homography
    R = quat_to_rotm(x(4:7));
    T = x(1:3);
    H = K * [R(:, 1:2) T];

    c = homography_project(H, X);

    x = round(c(1, :));
    y = round(c(2, :));

    if length(find(x < 1)) > 0 || ...
       length(find(y < 1)) > 0 || ...
       length(find(x > size(img, 2))) > 0 || ...
       length(find(y > size(img, 1))) > 0
        
        p = [];
        return;
    end
    
    idx = sub2ind(size(img), y, x);
    v = img(idx);
    p = reshape(v, size(p))';
end

function z = measureDiff(patchA, patchB)
    if (size(patchA) ~= size(patchB))
        z = 0;
        return;
    else
        a = int16(patchA);
        b = int16(patchB);

        z = abs(corr2(a, b));
        if z > 1 || isnan(z) % Some crazy value, like -Inf
            z = 1;
        end
    end
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
