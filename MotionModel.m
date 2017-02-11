classdef MotionModel < handle
    
    properties
        x % The best particle from the last iteration
        
        weights
        measurements
        particles
        
        params
        tagParams
        
        % Will transform a tag state by a particle
        transform
        
        % All the tags incorporated
        % in this motion model
        % this includes tags that are out of sight
        % This also stores the reference Patches for each tag
        modelledTags

        % A 1xn matrix storing which tags are visible
        visibleTags
    end
    
    methods
        function obj = MotionModel(parameters, transform)
            obj.transform = transform;
            
            obj.params = parameters;
            obj.weights = (1/parameters.num_particles) * ...
                ones([1 parameters.num_particles]);
            
            obj.particles = zeros([13 parameters.num_particles]);
            obj.particles(4, :) = ones([1 parameters.num_particles]);
            
            obj.modelledTags = {};
            obj.visibleTags = [];
        end

        % Will be called by the motion model to pass
        % along the coordinates
        function setTagParams(this, tagParams)
            this.tagParams = tagParams;
        end
        
        % Will set all the particles to a specific x
        function initializeParticlesTo(this, x)
            this.x = x;
            this.particles = repmat(x, 1, this.params.num_particles);
        end
        
        % Draw initial particles from a distribution
        function initializeParticlesRandom(this, mu, sigma)
            this.x = mu;
            for i=1:size(this.particles, 2)
                x = mu;
                
                % Add some noise        
                noise = sigma .* randn(12, 1);
                x(:) = x(:) + [noise(1:3); 1; 0; 0; 0; noise(7:12)];
                % Special case for the quaternion
                x(4:7) = qmult(mu(4:7)', rotvec_to_quat(noise(4:6)'))';

                this.particles(:, i) = x;
            end
        end
        
        function addTag(this, tag)
            tag.refPatch = [];
            this.modelledTags{length(this.modelledTags) + 1} = tag;
            this.visibleTags = [this.visibleTags 0];
        end
        
        function setTagVisible(this, id, visible)
            for i=1:length(this.modelledTags)
                if this.modelledTags{i}.id == id
                    this.visibleTags(i) = visible;
                    return
                end
            end
            % Could not find the tag
        end
        
        function loadTags(this, file)
            loadedTags = load_tag_config(file);
            for i=1:length(loadedTags)
                this.addTag(loadedTags{i});
            end
        end
        
        function updatePatches(this, detectedTagMap, img) 
            for i=1:length(this.modelledTags)
                id = this.modelledTags{i}.id;
                if (length(detectedTagMap.keys) >= id + 1) && ...
                    (length(detectedTagMap.keys{id + 1}) > 0) && ...
                    (detectedTagMap.keys{id + 1} > 0)
                    tag = detectedTagMap.values{detectedTagMap.keys{id + 1}};

                    % Update the corners
                    this.modelledTags{i}.refPatchCorners = tag.corners;

                    % Calculate the homography from the detected tag
                    % corners
                    tagSize = this.modelledTags{i}.size/2;
                    pin = [-tagSize(1), -tagSize(2); ...
                            tagSize(1), -tagSize(2); ...
                            tagSize(1),  tagSize(2);...
                           -tagSize(1),  tagSize(2)];                
                    H = homography_solve(pin, tag.corners);
                    % Scale to get the border as well + right units
                    s = (this.modelledTags{i}.size + this.modelledTags{i}.border) / 2;
                    
                    S = [s(1) 0 0; 0 s(2) 0; 0 0 1];
                    
                    tag.homography = H * S;
                    
                    % Extract the patch based on the tag homography
                    this.modelledTags{i}.refPatch = ... 
                        extract_patch(this.tagParams.patchSize, ...
                                      this.tagParams.coords, ...
                                      img, tag);
                    % make sure we note that the tag is visible
                    this.visibleTags(i) = 1;
                end
            end
        end

        
        function [tags, x] = process(this, img, detectedTagMap)
            % ---------------------  Resample -----------------------
            fprintf('*** Resampling ***\n');
            fprintf('--> Resampling');
            tic();

            [this.particles, this.weights] = ...
                resample_particles(size(this.particles, 2), ...
                                   this.particles, ...
                                   this.weights);

            fprintf('; Took: %f\n', toc());

            % --------------------  Propagate  ---------------------------
            
            fprintf('** Propagating ***\n');

            % Propagate by sampling from q
            fprintf('--> Propagating');
            tic();

            this.particles = q_smpl(this.particles, this.weights, 1, ...
                                    this.params.process_noise);
            fprintf('; Took: %f\n', toc());

            
            % -----------------------  Measure  ----------------------------
            fprintf('** Measuring ***\n');

            % First update the patches
            fprintf('--> Updating patches');
            tic();

            this.updatePatches(detectedTagMap, img);
            
            fprintf('; Took: %f\n', toc());           

            fprintf('--> Measuring particles');
            tic();

            Z = this.measureParticles(this.particles, img);
            
            fprintf('; Took: %f\n', toc());           

            fprintf('--> Transforming measurements');
            tic();
            
            
            % Convert the measurements to weights
            this.measurements = Z;
            W = this.transformMeasurements(Z);
            % this.measurements is really just the transformed weights
            
            % Update the old weights. We don't need to multiply as we just
            % resampled
            this.weights = W;
            
            % Finally, normalize the weights again
            weightSum = sum(this.weights);
            if weightSum == 0
                this.weights(1) = 1;
                weightSum = 1;
            end
            this.weights = this.weights / weightSum;

            fprintf('; Took: %f\n', toc());          

            fprintf('--> Selecting final particle and updating tags');
            tic();
            
            % Pick the particle with the highest weight
            %%{
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            
            % Measure which tags we should get rid of
            this.x = x;
            this.removeOccludedTags(this.x, img);

            tags = this.transformTags(this.x, detectedTagMap);
            fprintf('; Took: %f\n', toc());
        end
        
        % Utilities for measuring particles
        function Z = measureParticles(this, X, img)
            Z = zeros([1 size(X, 2)]);
            
            % Find all the tags which have a positive weight
            tags = {};
            for i=1:length(this.modelledTags)
                if this.visibleTags(i) > 0 ...
                        && length(this.modelledTags{i}.refPatch) > 0
                    tags{length(tags) + 1} = this.modelledTags{i};
                end
            end            
            
            % For every particle, go through each tag
            % and average the error
            parfor n=1:size(X, 2)
                x = X(:, n);
                
                z = 1;
                for i=1:size(tags, 2)
                    t = tags{i};
                    % Transform the state of the tag
                    t.state = this.transform(t.state, x);
                    t.homography = homography_from_state(this.tagParams.K, t);
                    
                    % Extract the patch based on the homography
                    % and calculate the error
                    p = extract_patch(this.tagParams.patchSize, ...
                                      this.tagParams.coords, ...
                                      img, t);
                                  
                    err = measure_patch_error(p, t.refPatch);
                    z = z * err;
                    %z
                end
                Z(n) = z;
                
                %pause(1);
            end
        end
        

        % Converts measurements to weights
        function W = transformMeasurements(this, Z) 
            W = exp(-this.params.lambda * Z);
            %W = this.params.lambda * Z;
        end

        function tags = transformTags(this, x, detectedTagMap)
            tags = {};
            
            c = 1;
            for i=1:length(this.modelledTags)
                if this.visibleTags(i) <= 0
                    continue;
                end
                tags{c} = this.modelledTags{i};
                % Adjust the state vector
                tags{c}.state = this.transform(tags{c}.state, x);
                
                
                id = this.modelledTags{i}.id;
                if (length(detectedTagMap.keys) >= id + 1) && ...
                   (length(detectedTagMap.keys{id + 1}) > 0) && ...
                   (detectedTagMap.keys{id + 1} > 0)
               
                    tags{c}.color = 'y';
                end
                c = c + 1;
            end
        end
        
        
        
        function removeOccludedTags(this, x, img)
            for i=1:length(this.modelledTags)
                t = this.modelledTags{i};
                
                w = this.visibleTags(i);
                if w > 0
                    % Transform the state
                    t.state = this.transform(t.state, x);
                    t.homography = homography_from_state(this.tagParams.K, t);
                    
                    % Measure the error
                    p = extract_patch(this.tagParams.patchSize, ...
                                      this.tagParams.coords, ...
                                      img, t);
                    
                    err = measure_patch_error(p, t.refPatch);
                    
                    % For error debugging
                    % err
                    
                    if err >= this.params.err_discard_threshold
                        this.visibleTags(i) = 0;
                    end
                end
            end            
        end
        
        
        function debug(this, img, x, fig1, fig2, fig3, fig4, fig5)
            sfigure(fig1);
            colormap(gray(255));
            
            % Draw the tags on figure1
            if length(this.modelledTags) > 0
                %p = this.modelledTags{32}.refPatch;
                %p = this.modelledTags{1}.refPatch;
                n = round(sqrt(length(this.modelledTags))) + 1;

                for i=1:length(this.modelledTags)
                    t = this.modelledTags{i};
                    if length(t.refPatch) > 0
                        h = subplot(n, n, i);
                        image(t.refPatch, 'Parent', h);
                        title(t.id);
                    end
                end
            end
            
            sfigure(fig2);
            g_x = linspace(-1, 1, 3);
            g_y = linspace(-1, 1, 3);
            g_z = linspace(-1, 1, 3);
            [s_x, s_y, s_z] = meshgrid(g_x, g_y, g_z);
            s_x = s_x(:);
            s_y = s_y(:);
            s_z = s_z(:);
            
            gen_particles = [ this.particles(1:3, :) [s_x s_y s_z]' ];
            
            gen_w = [ this.weights ...
                      zeros([1, size(s_x, 1)])];

            gen_m = [ this.measurements ...
                  zeros([1, size(s_x, 1)])];

            colormap('jet');
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_particles(3, :), 5, gen_w);
            
            sfigure(fig3);
            colormap('jet');
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_w, 5, gen_w);
            
            sfigure(fig4);
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_m, 5, gen_m);
                 
            sfigure(fig5);
            colormap(gray(255));

            n = round(sqrt(length(this.modelledTags))) + 1;
            for i=1:length(this.modelledTags)
                t = this.modelledTags{i};
                
                w = this.visibleTags(i);
                if w > 0
                    % Transform the state
                    t.state = this.transform(t.state, x);
                    t.homography = homography_from_state(this.tagParams.K, t);
                    
                    p = extract_patch(this.tagParams.patchSize, ...
                                      this.tagParams.coords, ...
                                      img, t);
                    
                    h = subplot(n, n, i);
                    image(p, 'Parent', h);
                    title(t.id);
                end
            end
        end        
    end
end
