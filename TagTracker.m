classdef TagTracker < TagSource
    properties
        tracker
        initialImage
        initialized
    end
    
    methods
        function obj = TagTracker()
            obj.tracker = vision.PointTracker();
            obj.initialized = false;
            obj.initialImage = [];
        end
        
        function track(this, tag)  
            points = [tag([1, 3, 5, 7]) tag([2, 4, 6, 8])];
            
            if ~this.initialized
                if max(size(this.initialImage)) > 0
                    this.tracker.initialize(points, this.initialImage);
                    this.initialized = true;
                end
                return;
            end
            
            this.tracker.setPoints(points);
        end
        
        function tags = process(this, img)
            tags = {};
            if ~this.initialized
                this.initialImage = img;
                return;
            end
            

            points = this.tracker.step(img);           
            tag = reshape(points', [8, 1]);
            tags{1} = tag;
        end
    end
end
