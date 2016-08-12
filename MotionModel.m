classdef MotionModel < handle
    %MOTIONMODEL 
    
    properties
        weights
        particles
        
        params
        tagParams
        
        % Will transform a tag state by a particle
        transform
        % Will take a modelled tag state, a transformed state
        % and give a particle
        invtransform
        
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
        function obj = MotionModel(parameters, transform, invtransform)
            obj.transform = transform;
            obj.invtransform = invtransform;
            
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
        function setParticles(this, x)
            this.particles = repmat(x, 1, this.params.num_particles);
        end
        
        function addTag(this, tag)
            tag.refPatch = [];
            this.modelledTags{length(this.modelledTags) + 1} = tag;
            this.tagWeights = [this.tagWeights 0];
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
        
        function tagsDetected(this, detectedTagMap)
            for i=1:length(this.modelledTags)
                id = this.modelledTags{i}.id;
                if detectedTagMap.isKey(id)
                    % The transformed tag
                    transTag = detectedTagMap(id);
                    tag = this.modelledTags{i};
                    
                    % Reset the particles
                    this.setParticles(this.invtransform(tag.state, transTag.state));
                    
                    % Update the reference patch
                    this.modelledTags{i}.refPatch = transTag.refPatch;
                end
            end
        end
        
        function debug(this, fig1)
            set(0, 'CurrentFigure', fig1)
            clf(fig1);
            p = this.modelledTags{1}.refPatch;
            imshow(p);
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
            tags = transformTags(this.modelledTags, x, this.transform);
            %tags{1}.state
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
                    %imshow(p)
                    %drawnow;
                    %pause(1)
                    z = z + w * err;
                end
                Z(n) = z;
            end
        end
    end
end

% The simplest model: individual tag-tracking
function tags = transformTags(modelledTags, x, transform)
    tags = cell(1, length(modelledTags));
    
    for i=1:length(modelledTags)
        tags{i} = modelledTags{i};
        % Adjust the state vector
        tags{i}.state = transform(tags{i}.state, x);
    end
end

