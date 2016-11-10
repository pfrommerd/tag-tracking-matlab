classdef AprilTrack < TagSource
    %PARTICLEFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % A cell array of motion models
        motionModels 
        
        params
    end
    
    methods
        function obj = AprilTrack(params)
            obj.motionModels = {};
            
            obj.params = params;
            
            % Pre-calculate the projection coordinates                        
            coordinates = zeros([params.patchSize 2]);
            
            for i=1:params.patchSize(2)
                for j=1:params.patchSize(1)
                    idx = [i / params.patchSize(2) - 0.5; ...
                           j / params.patchSize(1) - 0.5];

                    coordinates(j, i, :) = idx;
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
        
        function tags = process(this, img, detector_tags)
            tagMap.keys = {};
            tagMap.values = {};
            
            for i=1:length(detector_tags)
                tag = detector_tags{i};
                tagMap.keys{tag.id + 1} = i;
                tagMap.values{i} = tag;
            end
            
            tags = {};
            for i=1:length(this.motionModels)
                mm = this.motionModels{i};
                tags = [tags mm.process(img, tagMap)];
            end
        end
        function debug(this, fig1, fig2, fig3)
            if length(this.motionModels) > 0
                this.motionModels{1}.debug(fig1, fig2, fig3);
            end
        end
    end
end