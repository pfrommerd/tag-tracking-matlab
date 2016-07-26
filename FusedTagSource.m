classdef FusedTagSource < TagSource
    %FUSEDTAGSOURCE Combines both tracking and detecting
   
    properties
        detector
        tracker
    end
    
    methods
        function obj = FusedTagSource(tagSize, cameraParams, np)
            obj.detector = TagDetector(tagSize, cameraParams, np);
            obj.tracker = TagTracker(np);
        end
        
        function tags = process(this, img)
            tags = [];

            tracker_tags = this.tracker.process(img);
            if size(tracker_tags, 2) > 0
                tags = tracker_tags;
            end
            
            detector_tags = this.detector.process(img);
            if size(detector_tags, 2) > 0
                tags = detector_tags;
                this.tracker.track(tags);
            end
        end      
    end
    
end

