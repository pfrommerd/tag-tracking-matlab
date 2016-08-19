classdef MotionModel < handle
    %MOTIONMODEL 
    
    properties
        best_particle
        
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
            obj.best_particle = [0;0;0;1;0;0;0; 0;0;0; 0;0;0];
            
            obj.modelledTags = {};
            obj.tagWeights = [];
        end

        function setTagParams(this, tagParams)
            this.tagParams = tagParams;
        end
        
        % Will set all the particles to a specific x
        function setParticles(this, x)
            this.particles = repmat(x, 1, this.params.num_particles);
            this.best_particle = x;
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
        
        function tagsDetected(this, detectedTags)
            for n=1:length(detectedTags)
                transTag = detectedTags{n};
                
                foundTag = false;
                for i=1:length(this.modelledTags)
                    id = this.modelledTags{i}.id;
                    if transTag.id == id
                        tag = this.modelledTags{i};
                        % Reset the particles
                        this.setParticles(this.invtransform(tag.state, transTag.state));
                        % Update the reference patch
                        this.modelledTags{i}.refPatch = transTag.refPatch;
                        this.tagWeights(i) = 1;
                        
                        foundTag = true;
                        break;
                    end
                end
                
                if ~foundTag && this.params.auto_add_tags
                    tag(1).id = transTag.id;
                    tag(1).color = 'r';
                    tag(1).state = this.invtransform2(transTag.state, this.best_particle);
                    this.addTag(tag, 1);
                end
                
            end
        end
        
        function debug(this, fig1, fig2)
            set(0, 'CurrentFigure', fig1)
            clf(fig1);
            p = this.modelledTags{1}.refPatch;
            imshow(p);
            
            set(0, 'CurrentFigure', fig2)
            clf(fig2);
            g_x = linspace(-1, 1, 3);
            g_y = linspace(-1, 1, 3);
            g_z = linspace(0, 5, 3);
            [s_x, s_y, s_z] = meshgrid(g_x, g_y, g_z);
            s_x = s_x(:);
            s_y = s_y(:);
            s_z = s_z(:);
            
            gen_particles = [ this.particles(1:3, :) [s_x s_y s_z]' ];
            
            %gen_w = [ this.weights ];
            gen_m = [ this.convertToWeights(this.measurements) ...
                        ones([1, size(s_x, 1)])];

            colormap('parula');
            scatter3(gen_particles(1, :), gen_particles(2, :), ...
                     gen_particles(3, :), 5, -gen_m);
        end
        
        function tags = process(this, img)
            % Resample
            [this.particles, this.weights] = ...
                resample_particles(size(this.particles, 2), ...
                                    this.particles, ...
                                    this.weights);
            
            % Propagate
            this.particles = q_smpl(this.particles, this.weights, 1, ...
                                    this.params.process_noise, ...
                                    this.params.k, this.params.alpha);
            
            % Measure the new particles
            Z = this.measureParticles(this.particles, img);
            % Convert the measurements to weights
            W = this.convertToWeights(Z);
            this.measurements = W;
            % Update the old weights
            % using the formula w' = w * p(x'|x)/q(x'|x,y)
            % where q(x') = p(x') * 1/(k + alpha*w)
            % as particles with lower weights should have more noise
            % so p(x') = (k + alpha * w) * q(x')
            % and p(x')/q(x') = k + akpha * w
            
            this.weights = this.weights .* W .* ...
                    (this.params.alpha .* this.weights + this.params.k);
            
            % Finally, normalize the weights
            this.weights = this.weights / sum(this.weights);

            % Pick the particle with the highest weight
            [z, i] = max(this.weights);
            x = this.particles(:, i);
            
            % Measure which tags we should get rid of
            this.updateTagWeights(x, img);

            this.modelledTags
            tags = transformTags(this.modelledTags, this.tagWeights, x, this.transform);
            %tags{1}.state
        end
        
        function updateTagWeights(this, x, img)
            for i=1:size(this.modelledTags)
                t = this.modelledTags{i};
                w = this.tagWeights(i);
                
                if w > 0
                    % Measure the error
                    p = extract_patch(this.tagParams.K, ...
                                    this.tagParams.patchSize, ...
                                    this.tagParams.coords, ...
                                    img, this.transform(t.state, x));

                    err = measure_patch_error(p, t.refPatch);
                    if err > 0.5
                        this.tagWeights(i) = 0;
                    end
                end
            end            
        end
        
        % Converts measurements to weights
        function W = convertToWeights(this, Z)
            W = exp(-this.params.lambda * Z);
        end
        
        % Utilities for measuring particles
        function Z = measureParticles(this, X, img)
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
                                    img, this.transform(t.state, x));
                                    
                    err = measure_patch_error(p, t.refPatch);
                    z = z + w * err;
                end
                Z(n) = z;
            end
        end
    end
end

% The simplest model: individual tag-tracking
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

