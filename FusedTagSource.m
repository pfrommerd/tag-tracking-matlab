classdef FusedTagSource < TagSource
    %FUSEDTAGSOURCE Combines both tracking and detecting
   
    properties
        detector
        tracker
    end
    
    methods
        function obj = FusedTagSource(tagSize, K, np)
            obj.detector = TagDetector(tagSize, K, np);
            obj.tracker = TagTracker(np);
        end
        
        function tags = process(this, img)
            %%{
            tracker_tags = this.tracker.process(img);
            if size(tracker_tags, 2) > 0
                tags = tracker_tags;
            end
            %}
            detector_tags = this.detector.process(img);
            if size(detector_tags, 2) > 0
                tags = detector_tags;
                this.tracker.set(tags);
            end
            
            tags = merge(detector_tags, tracker_tags);
        end

    end
    
end


% A function which will merge two sets
% of tags, preferring tags in tagA over tagB
function tags = merge(tagsA, tagsB)
    tags = tagsA;

    for i=1:length(tagsB)
        t = tagsB{i};
        
        found = false;
        for j=1:length(tagsA)
            if tagsA{j}.id == tagsB{i}.id
                found = true;
                break;
            end
        end
        
        if ~found
            tags{length(tags) + 1} = t;
        end
    end
end
