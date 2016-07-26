classdef TagTracker < TagSource
    properties
        tracker
        initialImage
        initialized
        
        num_pts
    end
    
    methods
        function obj = TagTracker(np)
            obj.tracker = vision.PointTracker();
            obj.initialized = false;
            obj.initialImage = [];
            obj.num_pts = np;
        end
        
        function track(this, tags)
            points = reshape(tags, [2, size(tags, 1) * size(tags, 2) / 2])';
            
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
            tags = [];
            if ~this.initialized
                this.initialImage = img;
                return;
            end          

            points = this.tracker.step(img);    
            tags = points_to_pvec(points, this.num_pts);            
        end
    end
end
