classdef AprilTrack < TagSource
    %PARTICLEFILTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % A cell array of motion models
        motionModels 
        
        tagParams
    end
    
    methods
        function obj = AprilTrack(tagParams)
            obj.motionModels = {};
            
            obj.tagParams = tagParams;
            
            % Pre-calculate the projection coordinates                        
            coordinates = zeros([tagParams.patchSize 2]);
            
            for i=1:tagParams.patchSize(2)
                for j=1:tagParams.patchSize(1)
                    idx = 2* [i / tagParams.patchSize(2) - 0.5; ...
                              j / tagParams.patchSize(1) - 0.5];
                    coordinates(j, i, :) = idx;
                end
            end
            
            obj.tagParams.coords = reshape(permute(coordinates, [3, 2, 1]), ...
                                        2, tagParams.patchSize(2) * tagParams.patchSize(1));
            obj.tagParams.coords = [obj.tagParams.coords; ones([1 size(obj.tagParams.coords, 2)])];
        end
        
        function addMotionModel(this, mm)
            this.motionModels{length(this.motionModels) + 1} = mm;
            mm.setTagParams(this.tagParams);
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
        function debug(this, img, fig1, fig2, fig3, fig4, fig5)
            if length(this.motionModels) > 0
                this.motionModels{1}.debug(img, fig1, fig2, fig3, fig4, fig5);
            end
        end
    end
end