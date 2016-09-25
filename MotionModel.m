classdef MotionModel < handle
    %MOTIONMODEL 
    
    properties
        weights
        measurements
        particles
        
        params
        tagParams
        
        % Will transform a tag state by a particle
        transform
        % Will take a modelled tag state, a transformed state
        % and give a particle
        invtransform
        % Will take a transformed state and a particle and give
        % a modelled tag state
        invtransform2
        
        % All the tags incorporated
        % in this motion model
        % this includes tags that are out of sight
        % This also stores the reference Patches for each tag
        modelledTags

        % A 1xn matrix storing how much each tag should be weighted
        % into the error
        tagWeights
    end
    
    methods
        function obj = MotionModel(parameters, transform, invtransform, invtransform2)
            obj.transform = transform;
            obj.invtransform = invtransform;
            obj.invtransform2 = invtransform2;
            
            obj.params = parameters;
            obj.weights = (1/parameters.num_particles) * ...
                            ones([1 parameters.num_particles]);
                        
            obj.particles = zeros([13 parameters.num_particles]);
            obj.particles(4, :) = ones([1 parameters.num_particles]);
            
            obj.modelledTags = {};
            obj.tagWeights = [];
        end

        function setTagParams(this, tagParams)
            this.tagParams = tagParams;
        end
        
        % Will set all the particles to a specific x
        function initializeParticlesTo(this, x)
            this.particles = repmat(x, 1, this.params.num_particles);
        end
        
        function initializeParticlesRandom(this, mu, sigma)

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
        
        
        function addTag(this, tag, weight)
            tag.refPatch = [];
            this.modelledTags{length(this.modelledTags) + 1} = tag;
            this.tagWeights = [this.tagWeights weight];
        end
        
        function setTagWeight(this, id, weight)
            for i=1:length(this.modelledTags)
                if this.modelledTags{i}.id == id
                    this.tagWeights(i) = weight;
                    return
                end
            end
            % Could not find the tag
        end
        
        function loadTags(this, file)
            text = textread(file);
            for i=1:size(text, 1)
                tagInfo = text(i, :);
                
                tag(1).id = tagInfo(1);
                tag(1).color = 'r';
                tag(1).size = [0.1635 0.1635];
                tag(1).border = [0.02 0.02];

                pos = (0.001 .* [tagInfo(3) tagInfo(4) tagInfo(5)])';
                rot = vrrotvec_to_quat([tagInfo(6) tagInfo(7) tagInfo(8) tagInfo(9)])';
                tag(1).state = [pos; rot; 0; 0; 0; 0; 0; 0];
                
                if tag(1).id == 23
                %if true
                    this.addTag(tag, 0);
                end
            end
        end
        
        function updatePatches(this, detectedTagMap, img) 
            for i=1:length(this.modelledTags)
                id = this.modelledTags{i}.id;
                if detectedTagMap.isKey(id)
                    transTag = detectedTagMap(id);
                    tag = this.modelledTags{i};

                    % Update the reference patch
                    % First compute the state from the corners
                    tagSize = tag.size/2;
                    pin = [-tagSize(1), -tagSize(2); ...
                            tagSize(1), -tagSize(2); ...
                            tagSize(1),  tagSize(2);...
                           -tagSize(1),  tagSize(2)];

                    H = homography_solve(pin, transTag.corners);
                    % Extract the rotation and translation
                    [R, T] = homography_extract_pose(this.tagParams.K, H);

                    % Comute a quaternion from a rot mat
                    transState = [T; rotm_to_quat(R)'; 0; 0; 0; 0; 0; 0];

                    this.modelledTags{i}.refPatchCorners = transTag.corners;

                    % Extract the patch
                    this.modelledTags{i}.refPatch = ... 
                        extract_patch(this.tagParams.K, ...
                                         this.tagParams.patchSize, ...
                                         this.tagParams.coords, ...
                                         img, tag, transState);
                    
                    this.tagWeights(i) = 1;
                end
            end
        end

        
        function tags = process(this, img, detectedTagMap)
            % Resample
            [this.particles, this.weights] = ...
                resample_particles(size(this.particles, 2), ...
                                    this.particles, ...
                                    this.weights);
            
            % Propagate by sampling from q
            this.particles = q_smpl(this.particles, this.weights, 1, ...
                                    this.params.process_noise, ...
                                    this.params.k, this.params.alpha);
                                
            % Measure the new particles
            % First update the patches
            this.updatePatches(detectedTagMap, img);
            
            Z = this.measureParticles(this.particles, img, detectedTagMap);
            this.measurements = Z;
            
            % Convert the measurements to weights
            W = convertToWeights(this.params, Z);
            
            % Update the old weights
            % using the formula w' = w * p(x'|x)/q(x'|x,y)
            % where q(x') = p(x') * 1/(k + alpha*w)
            % as particles with lower weights should have more noise
            % so p(x') = (k + alpha * w) * q(x')
            % and p(x')/q(x') = k + akpha * w
            
            this.weights = this.weights .* W .* ...
                    (this.params.alpha .* this.weights + this.params.k);
            
            % Finally, normalize the weights again
            weightSum = sum(this.weights);
            if weightSum == 0
                this.weights(1) = 0.0001;
                weightSum = 0.0001;
            end
            this.weights = this.weights / weightSum;
            
            % Pick the particle with the highest weight
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            % Measure which tags we should get rid of
            this.updateTagWeights(x, img);

            tags = transformTags(this.modelledTags, this.tagWeights, x, this.transform);
        end
        
        % Utilities for measuring particles
        function Z = measureParticles(this, X, img, detectedTagMap)
            keys(detectedTagMap)
            Z = zeros([1 size(X, 2)]);
            
            % Find all the tags which have a positive weight
            tags = {};
            weights = [];
            for i=1:length(this.modelledTags)
                if this.tagWeights(i) > 0 ...
                        && length(this.modelledTags{i}.refPatch) > 0
                    tags{length(tags) + 1} = this.modelledTags{i};
                    weights = [ weights this.tagWeights(i) ];
                end
            end
            % Normalize the weights
            weights = weights ./ sum(weights);
            
            % For every particle, go through each tag
            % and average the error
            for n=1:size(X, 2)
                x = X(:, n);
                
                z = 0;
                for i=1:size(tags)
                    t = tags{i};
                    w = weights(i);
                    
                    % Measure the error
                    p = extract_patch(this.tagParams.K, ...
                                    this.tagParams.patchSize, ...
                                    this.tagParams.coords, ...
                                    img, t, this.transform(t.state, x));
                    
                    err = measure_patch_error(p, t.refPatch);
                    %{
                    pause(2);
                    figure(4);
                    imshow(p);
                    err
                    %}          
                    z = z + w * err;

                    if detectedTagMap.isKey(t.id)
                        % Calculate tag error
                        projTags = project_tags(this.tagParams.K, {t});
                        projTag = projTags{1};
                        
                        corners_diff = (projTag.corners - detectedTagMap(t.id).corners);
                        %corners_diff
                        corner_err = this.params.rho * sum(sum(corners_diff .* corners_diff)); 
                        
                        z = z + w * corner_err
                    end
                    %z
                end
                Z(n) = z;
            end
        end
        
                
        function updateTagWeights(this, x, img)
            for i=1:length(this.modelledTags)
                t = this.modelledTags{i};
                w = this.tagWeights(i);
                if w > 0
                    % Measure the error
                    p = extract_patch(this.tagParams.K, ...
                                    this.tagParams.patchSize, ...
                                    this.tagParams.coords, ...
                                    img, t, this.transform(t.state, x));

                    err = measure_patch_error(p, t.refPatch);
                    if err > this.params.err_discard_threshold
                        this.tagWeights(i) = 0;
                    else
                        %this.tagWeights(i) = this.tagWeights(i) * exp(err - this.params.err_dec_factor);
                    end
                end
            end            
        end
        
        
        function debug(this, fig1, fig2, fig3)
            set(0, 'CurrentFigure', fig1)
            clf(fig1);
            if length(this.modelledTags) > 0
                %p = this.modelledTags{32}.refPatch;
                p = this.modelledTags{1}.refPatch;
                imshow(p);
            end
            
            set(0, 'CurrentFigure', fig2);
            clf(fig2);
            g_x = linspace(-1, 1, 3);
            g_y = linspace(-1, 1, 3);
            g_z = linspace(-1, 1, 3);
            [s_x, s_y, s_z] = meshgrid(g_x, g_y, g_z);
            s_x = s_x(:);
            s_y = s_y(:);
            s_z = s_z(:);
            
            gen_particles = [ this.particles(1:3, :) [s_x s_y s_z]' ];
            
            gen_m = [ this.weights ...
                      zeros([1, size(s_x, 1)])];

            colormap('parula');
            caxis auto;
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_particles(3, :), 5, gen_m);
                 
            set(0, 'CurrentFigure', fig3);
            clf(fig3);
            colormap('parula');
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_m, 5, gen_m);
        end        
    end
end

% Converts measurements to weights
function W = convertToWeights(params, Z)
    W = exp(-params.lambda * Z);
end

function tags = transformTags(modelledTags, tagWeights, x, transform)
    tags = {};
    
    c = 1;
    for i=1:length(modelledTags)
        if tagWeights(i) <= 0
            continue;
        end
        tags{c} = modelledTags{i};
        % Adjust the state vector
        tags{c}.state = transform(tags{c}.state, x);
        c = c + 1;
    end
end