classdef TagDetector < TagSource
    properties
        num_pts

        tagSize
        params
        
        K
    end
    
    methods
        function detector = TagDetector(tagSize, K, np)
            detector.tagSize = tagSize;
            detector.K = K;
            params = [K(1, 1) K(2, 2) ...
                      K(1, 3) K(2, 3)];            
            detector.params = params;
            detector.num_pts = np;
        end
        
        function tags = process(this, img)
            results = find_apriltags(img, this.tagSize, this.params);
            resultsSize = size(results);

            tags = {};
            for i = 1:resultsSize(2)
                r = results(i);
                
                tag.id = r.id;
                tag.color = 'r';
                tag.corners = r.corners';
                tags{i} = tag;
            end
            
        end
    end
    
end

