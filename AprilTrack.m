classdef AprilTrack < TagSource
    %PARTICLEFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        detector
        % A cell array of motion models
        motionModels 
        
        params
    end
    
    methods
        function obj = AprilTrack(params)
            obj.detector = TagDetector(params.K, params.tagSize);
            obj.motionModels = {};
            
            obj.params = params;
            
            % Pre-calculate the projection coordinates                        
            totalTagSize = params.patchTagSize;
            coordinates = zeros([params.patchSize 2]);
            for i=1:params.patchSize(2)
                for j=1:params.patchSize(1)
                    idx = [i / params.patchSize(2) - 0.5; ...
                           j / params.patchSize(1) - 0.5];

                    coordinates(j, i, :) = [idx(1) * totalTagSize(1); ...
                                            idx(2) * totalTagSize(2)];
                end
            end
            obj.params.coords = reshape(permute(coordinates, [3, 2, 1]), ...
                                    2, params.patchSize(2) * params.patchSize(1));
            obj.params.coords = [obj.params.coords; ones([1 size(obj.params.coords, 2)])];
        end
        
        function addMotionModel(this, mm)
            mm.setTagParams(this.params);
            this.motionModels{length(this.motionModels) + 1} = mm;
        end
        
        function tags = process(this, img)
            detector_tags = this.detector.process(img);
                                
            if length(detector_tags) > 0
                this.tagsDetected(detector_tags, img);
            end
            
            
            tags = {};
            for i=1:length(this.motionModels)
                mm = this.motionModels{i};
                tags = [tags mm.process(img)];
            end
            %tags = detector_tags;
        end
        
        function tagsDetected(this, tags, img)
            % Create a bunch of reference patches
            % A map from tagID -> tag
            detectedTagsMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            for i=1:length(tags)
                t = tags{i};
                p = extract_patch(this.params.K, this.params.patchSize, ...
                                    this.params.coords, img, t.state);
                t = tags{i};
                t.refPatch = p;
                detectedTagsMap(t.id) = t;
            end
            
            for i=1:length(this.motionModels)
                model = this.motionModels{i};
                model.tagsDetected(detectedTagsMap);
            end
        end
    end
end