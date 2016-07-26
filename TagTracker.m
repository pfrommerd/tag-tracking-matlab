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
        
        function track(this, tags)  
            x = tags([1, 3, 5, 7], :);
            x = x(:);
            
            y = tags([2, 4, 6, 8], :);
            y = y(:);
            
            points = [x y];
            
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
            tags = reshape(points', [8, size(points, 1) / 4]);
        end
    end
end
