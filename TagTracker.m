classdef TagTracker < TagSource
    properties
        tracker
        initialImage
        initialized
        
        K
        tagSize
    end
    
    methods
        function obj = TagTracker()
            obj.tracker = vision.PointTracker();
            obj.initialized = false;
            obj.initialImage = [];
            obj.K = [];
        end
        
        function track(this, K, tagSize, tag)
            this.K = K;
            this.tagSize = tagSize;
            
            points = projectTag(K, tagSize, tag)';
            
            if ~this.initialized
                if max(size(this.initialImage)) > 0
                    this.tracker.initialize(points, this.initialImage);
                    this.initialized = true;
                end
                return;
            end
            
            this.tracker.setPoints(points);
        end
        
        function tags = process(this, img, lastTag)
            tags = {};
            if ~this.initialized
                this.initialImage = img;
                return;
            end
            

            points = this.tracker.step(img);
            
            tagSize = this.tagSize/2;
            pin = [-tagSize, tagSize; ...
                     tagSize tagSize; ...
                     tagSize -tagSize;...
                     -tagSize -tagSize];
            H = homography_solve(pin, points);
            
            [R, T] = homography_extract_pose(this.K, H);
            
            % Comute a quaternion from a rot mat
            rot = rotm_to_quat(R)';

            delta_q = qmult(qinv(lastTag(4:7)'), rot')';


            tags{1} = [ T; rot; T- lastTag(1:3); quat_to_rotvec(delta_q')'];
        end
    end
end
